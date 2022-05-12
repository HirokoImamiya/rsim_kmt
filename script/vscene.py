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
from tf import transformations
from rviz_tools_py import rviz_tools
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from rovi_utils import tflib

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
pub_wp=rospy.Publisher("/vscene/floats",numpy_msg(Floats),queue_size=1)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
rospy.on_shutdown(cleanup_node)

markers=rviz_tools.RvizMarkers('world', 'vscene_marker')
Points=np.array([]).reshape((-1,3))
Points_lock=False

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

rospy.sleep(2)

while not rospy.is_shutdown():
  showModel('marud',frame='marud',color=(0.8,0.8,0.8))
  showModel('tatep',color=(0.7,0.7,0.7),x=-100)
  showModel('tatep',color=(0.7,0.7,0.7),x=-500)
  Points_lock=True
  rospy.Rate(3).sleep()
