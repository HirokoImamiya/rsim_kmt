#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import copy 

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

Config={
  "model_frame_ids":[]
}

Param={
  "pcd_clear":True,
  "tf_update":True,
  "marud1":{
    "name":"marud",
    "show":True,
    "ini_z":0,
    "uf":"uf1",
    "xyz":[0,0,0],
    "rpy":[0,0,0],
  },
  "marud2":{
    "name":"marud",
    "show":True,
    "ini_z":400,
    "uf":"uf1",
    "xyz":[0,0,0],
    "rpy":[0,0,0]
  },
  "marud3":{
    "name":"marud",
    "show":True,
    "ini_z":800,
    "uf":"uf1",
    "xyz":[0,0,0],
    "rpy":[0,0,0]
  },
  "circle1":{
    "name":"circle",
    "show":True,
    "ini_z":0,
    "uf":"uf2",
    "xyz":[0,0,0],
    "rpy":[0,0,0]
  },
  "circle2":{
    "name":"circle",
    "show":True,
    "ini_z":200,
    "uf":"uf2",
    "xyz":[0,0,0],
    "rpy":[0,0,0]
  },
  "circle3":{
    "name":"circle",
    "show":True,
    "ini_z":400,
    "uf":"uf2",
    "xyz":[0,0,0],
    "rpy":[0,0,0]
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

def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

def cb_redraw(msg):
  f=Floats()
  f.data=np.ravel(Points)
  pub_wp.publish(f)

def cb_update(msg):
  global Param
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  update=False
  if msg.data:
    update=True
  Param["tf_update"]=update
  Param["pcd_clear"]=True

def moveModel(frame,prm):
  pos=prm["xyz"]+prm["rpy"]
  rot=R.from_euler('xyz',pos[3:6],degrees=True)
  Tm=np.eye(4)
  Tm[:3,:3]=rot.as_matrix()
  Tm[:3,3]=np.array(pos[:3]).T
  bTu=getRT("base",prm["uf"])
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.child_frame_id=frame
  tf.transform=tflib.fromRT(bTu.dot(Tm))
  pub_tf.publish(tf)
  rospy.sleep(0.01)

def showModel(name,frame='world',color=(0.8,0.8,0.8),x=0,y=0,z=0,theta=0,pcdup=False):
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
  if not pcdup:
    thispath=subprocess.getoutput("rospack find rsim_kmt")
    pcd=o3d.io.read_point_cloud(thispath+'/model/'+name+'.ply')
    pcd=pcd.transform(wTu.dot(uT))
    Points=np.vstack((Points,np.array(pcd.points)))

# Initialize the ROS Node
rospy.init_node('vscene', anonymous=False, log_level=rospy.INFO, disable_signals=False)
###Load params
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print("config exception:",e.args)
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
pub_wp=rospy.Publisher("/vscene/floats",numpy_msg(Floats),queue_size=1)
pub_tf=rospy.Publisher("/update/config_tf",TransformStamped,queue_size=1);
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.Subscriber("/vscene/update",Bool,cb_update)
rospy.on_shutdown(cleanup_node)

markers=rviz_tools.RvizMarkers('world', 'vscene_marker')
Points=np.array([]).reshape((-1,3))

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
  nowParam=copy.deepcopy(Param)
  Param["pcd_clear"]=False
  pcd_lock=True
  if nowParam["pcd_clear"]:
    Points=np.array([]).reshape((-1,3))
    pcd_lock=False
    if nowParam["tf_update"]:
      for frame in Config["model_frame_ids"]:
        if frame in nowParam:
          moveModel(frame,nowParam[frame])

  for frame in Config["model_frame_ids"]:
    if frame in nowParam:
      prm=nowParam[frame]
      if prm["show"]:
        showModel(prm["name"],frame=frame,color=(0.8,0.8,0.8),z=prm["ini_z"],pcdup=pcd_lock)
  showModel('tatep',color=(0.7,0.7,0.7),x=-100,pcdup=pcd_lock)
  showModel('tatep',color=(0.7,0.7,0.7),x=-500,pcdup=pcd_lock)
  if not pcd_lock:
    cb_redraw(mTrue)
  rospy.Rate(3).sleep()
