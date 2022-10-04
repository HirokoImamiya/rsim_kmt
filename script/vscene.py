#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# ROS includes
import roslib
import rospy
import tf
import tf2_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from tf import transformations
from rviz_tools_py import rviz_tools
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from rovi_utils import tflib

Param={
  "uf":"uf1",
  "frame":"marud1",
  "xyz":[0,0,0],
  "rpy":[0,0,0],
  "marud1":{
    "show": True
  },
  "marud2":{
    "show": True
  },
  "marud3":{
    "show": True
  },
  "circle1":{
    "show": True
  },
  "circle2":{
    "show": True
  },
  "circle3":{
    "show": True
  }
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print('frame not found',ref)
    RT=np.eye(4)
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def mov(pos):
  print("vscene move",pos,Param["uf"])
  rot=R.from_euler('xyz',pos[3:6],degrees=True)
  Tm=np.eye(4)
  Tm[:3,:3]=rot.as_matrix()
  Tm[:3,3]=np.array(pos[:3]).T
  bTu=getRT("base",Param["uf"])
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.child_frame_id=Param["frame"]
  tf.transform=tflib.fromRT(bTu.dot(Tm))
  pub_tf.publish(tf);

def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

def cb_redraw(msg):
  f=Floats()
  f.data=np.ravel(Points)
  pub_wp.publish(f)

def cb_mov(msg):
  global Param, Points_clear
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  mov(Param["xyz"]+Param["rpy"])
  Points_clear=True
  pub_moved.publish(mTrue)

def cb_update(msg):
  global Param, Points_clear
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  Points_clear=True

def showModel(name,frame='world',color=(0.8,0.8,0.8),x=0,y=0,z=0,theta=0):
  global Points
  mesh='package://rsim_kmt/model/'+name+'.stl'
  wTu=getRT('world',frame)
  scale=Vector3(1,1,1)
  uT=np.eye(4)
  uT[0,3]=x
  uT[1,3]=y
  uT[2,3]=z
  uT[:3,:3]=R.from_euler('Z',theta,degrees=True).as_matrix()
  markers.publishMesh(wTu.dot(uT),mesh,color,scale, 0.5)
  if not Points_lock:
    thispath=subprocess.getoutput("rospack find rsim_kmt")
    pcd=o3d.io.read_point_cloud(thispath+'/model/'+name+'.ply')
    pcd=pcd.transform(wTu.dot(uT))
    Points=np.vstack((Points,np.array(pcd.points)))

# Initialize the ROS Node
rospy.init_node('vscene', anonymous=False, log_level=rospy.INFO, disable_signals=False)
###Load params
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
pub_wp=rospy.Publisher("/vscene/floats",numpy_msg(Floats),queue_size=1)
pub_moved=rospy.Publisher("/vscene/moved",Bool,queue_size=1);
pub_tf=rospy.Publisher("/update/config_tf",TransformStamped,queue_size=1);
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.Subscriber("/vscene/mov",Bool,cb_mov)
rospy.Subscriber("/vscene/update",Bool,cb_update)
rospy.on_shutdown(cleanup_node)

markers=rviz_tools.RvizMarkers('world', 'vscene_marker')
Points=np.array([]).reshape((-1,3))
Points_lock=False
Points_clear=False

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

####Bools
mTrue = Bool()
mTrue.data = True
mFalse = Bool()

rospy.sleep(2)

while not rospy.is_shutdown():
  if Points_clear:
    Points=np.array([]).reshape((-1,3))
    Points_lock=False
    Points_clear=False

  if Param["marud1"]["show"]:
    showModel('marud',frame='marud1',color=(0.8,0.8,0.8))
  if Param["marud2"]["show"]:
    showModel('marud',frame='marud2',color=(0.8,0.8,0.8),z=400)
  if Param["marud3"]["show"]:
    showModel('marud',frame='marud3',color=(0.8,0.8,0.8),z=800)
  if Param["circle1"]["show"]:
    showModel('circle',frame='circle1',color=(0.8,0.8,0.8))
  if Param["circle2"]["show"]:
    showModel('circle',frame='circle2',color=(0.8,0.8,0.8),z=200)
  if Param["circle3"]["show"]:
    showModel('circle',frame='circle3',color=(0.8,0.8,0.8),z=400)
  showModel('tatep',color=(0.7,0.7,0.7),x=-100)
  showModel('tatep',color=(0.7,0.7,0.7),x=-500)
  if not Points_lock:
    cb_redraw(mTrue)
  Points_lock=True
  rospy.Rate(3).sleep()
