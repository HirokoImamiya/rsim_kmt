#!/usr/bin/env python3

# Python includes
import numpy as np
import subprocess
import yaml

# ROS includes
import roslib
import rospy
import time
import tf2_ros
import cv2
import json
from std_msgs.msg import Bool
from std_msgs.msg import String
from rovi_utils import tflib
from rovi_utils.srv import TextFilter
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

Config={
  "sub_timeout": 30.0,
  "base_frame_id": "base",
  "source_frame_id": "camera/master0",
  "target_frame_id": "camera/capture0/solve0"
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def seq_start():
  global seq_exec
  seq_exec = True
  rospy.loginfo("auto: ### vt start ###")

def seq_finished():
  global seq_exec
  seq_exec = False
  rospy.loginfo("auto: ### vt finished ###")

def is_seq_exec():
  return seq_exec

def set_clear():
# cropper の clear は応答ないため、通知のみ
  rospy.loginfo("auto: clear")
  pub_clear.publish(mTrue)
  return True

def set_solve():
  result = False
  msg_type = ''
  rospy.loginfo("auto: solve start")
  pub_solve.publish(mTrue)
  try:
    msg = rospy.wait_for_message('~solved', Bool, timeout=Config['sub_timeout'])
    rospy.loginfo("auto: solve finished msg result=%s", msg.data)
    if msg.data:
      result = True
      msg_type = 'ok'
    else:
      msg_type = 'ng'
  except rospy.ROSInterruptException as e:
    msg_type = 'shutdown interrupts'
  except rospy.ROSException as e:
    msg_type = 'timeout'
  if msg_type:
    rospy.loginfo("auto: solve finished result=%s", msg_type)
  return result

def set_robo_move(prm, capture):
  result = False
  msg_type = ''
  rospy.loginfo("auto: robo move start")
  rospy.set_param('/vrobo/xyz', prm['xyz'])
  rospy.set_param('/vrobo/rpy', prm['rpy'])
  msg = Bool()
  msg.data = capture
  pub_rmove.publish(msg)
  try:
    msg = rospy.wait_for_message('~rmoved', Bool, timeout=Config['sub_timeout'])
    if msg.data:
      result = True
      msg_type = 'ok'
    else:
      msg_type = 'ng'
  except rospy.ROSInterruptException as e:
    msg_type = 'shutdown interrupts'
  except rospy.ROSException as e:
    msg_type = 'timeout'
  if msg_type:
    rospy.loginfo("auto: robo move finished result=%s", msg_type)
  return result

def set_robo_path(path, prm, capture):
  result = False
  msg_type = ''
  rospy.loginfo("auto: robo path move start")
  if 'ip'in prm:
    rospy.set_param('/vrobo/' + path + '/ip', prm['ip'])
  if 'xyz'in prm:
    rospy.set_param('/vrobo/' + path + '/xyz', prm['xyz'])
  if 'rpy'in prm:
    rospy.set_param('/vrobo/' + path + '/rpy', prm['rpy'])
  if 'var'in prm:
    rospy.set_param('/vrobo/' + path + '/var', prm['var'])
  if 'pitch'in prm and (path == 'path_interval'):
    rospy.set_param('/vrobo/' + path + '/pitch', prm['pitch'])
  if 'radius'in prm:
    rospy.set_param('/vrobo/' + path + '/radius', prm['radius'])
  msg = Bool()
  msg.data = capture
  exec('pub_' + path + '.publish(msg)')
  try:
    msg = rospy.wait_for_message('~inpos', Bool, timeout=Config['sub_timeout'])
    if msg.data:
      result = True
      msg_type = 'ok'
    else:
      msg_type = 'ng'
  except rospy.ROSInterruptException as e:
    msg_type = 'shutdown interrupts'
  except rospy.ROSException as e:
    msg_type = 'timeout'
  if msg_type:
    rospy.loginfo("auto: robo parh move finished result=%s", msg_type)
  return result

def robo_init_move():
  result = False
  if 'initpos'in Param:
    prm = Param['initpos']
    if 'xyz' in prm and 'rpy' in prm:
      rospy.loginfo("auto: robot init move request")
      if 'uf' in prm:
        rospy.set_param('/vrobo/uf', prm['uf'])
      result = set_robo_move(prm, False)
    else:
      rospy.logerr("auto: robot init position format error")
  else:
    result = True
  return result

def robo_one_move(prm):
  result = False
  if 'pos' in prm and 'xyz' in prm and 'rpy' in prm:
    rospy.loginfo("auto: capture position request pos=%d", prm['pos'])
    if 'uf' in prm:
      rospy.set_param('/vrobo/uf', prm['uf'])
    result = set_robo_move(prm, True)
  else:
    rospy.logerr("auto: capture position format error")
  return result

def robo_path_move(prm):
  result = False
  if 'pos' in prm and 'type' in prm and 'path' in prm:
    rospy.loginfo("auto: capture path request pos=%d path=%s", prm['pos'], prm['type'])
    if 'uf' in prm:
      rospy.set_param('/vrobo/uf', prm['uf'])
    result = set_robo_path(prm['type'], prm['path'], True)
  else:
    rospy.logerr("auto: capture path format error")
  return result

def recipe_load():
  result = False
  msg_type = ''
  if 'recipe' in Param:
    rospy.loginfo("auto: recipe load name=%s", Param['recipe'])
    pub_load.publish(Param['recipe'])
    try:
      msg = rospy.wait_for_message('~loaded', Bool, timeout=Config['sub_timeout'])
      if msg.data:
        result = True
        msg_type = 'ok'
      else:
        msg_type = 'ng'
    except rospy.ROSInterruptException as e:
      msg_type = 'shutdown interrupts'
    except rospy.ROSException as e:
      msg_type = 'timeout'
    if msg_type:
      rospy.loginfo("auto: recipe load finished result=%s", msg_type)
  else:
    rospy.logerr("auto: recipe cannot set")
  return result

def set_all_model_show(show, tfup):
  for prm in Model:
    rospy.set_param('/vscene/' + prm['model'] + '/show' ,show)
  rospy.loginfo("auto: set_all_model_show show=%s", show)
  msg = Bool()
  msg.data = tfup
  pub_mupdae.publish(msg)
  rospy.sleep(5)

def set_model_show(model, show, tfup):
  rospy.set_param('/vscene/' + model + '/show' ,show)
  rospy.loginfo("auto: set_model_show model=%s show=%s", model, show)
  msg = Bool()
  msg.data = tfup
  pub_mupdae.publish(msg)
  rospy.sleep(5)

def set_model_end_show(prm):
  if 'model'in prm:
    show = False
    if 'end_show' in prm:
      show = prm['end_show']
    set_model_show(prm['model'], show, False)

def change_model_pos():
  result = False
  pre_prm = []
  for mprm in Model:
    result = True
    if 'modelpos'in mprm:
      prm = mprm['modelpos']
      if 'xyz' not in prm and 'xyz' in pre_prm:
        prm['xyz'] = pre_prm['xyz']
      if 'rpy' not in prm and 'rpy' in pre_prm:
        prm['rpy'] = pre_prm['rpy']
      if 'frame' in prm and 'xyz' in prm and 'rpy' in prm:
        if 'range_x' in prm:
          prm['xyz'][0] = prm['range_x'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_x']
        if 'range_y' in prm:
          prm['xyz'][1] = prm['range_y'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_y']
        if 'range_z' in prm:
          prm['xyz'][2] = prm['range_z'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_z']
        if 'range_rx' in prm:
          prm['rpy'][0] = prm['range_rx'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_rx']
        if 'range_ry' in prm:
          prm['rpy'][1] = prm['range_ry'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_ry']
        if 'range_rz' in prm:
          prm['rpy'][2] = prm['range_rz'] * np.random.rand()   # (b-a)*np.random.rand()+a a=0, b=prm['range_rz']
        rospy.loginfo("auto: model move request model=%s xyz=%s, rpy=%s", mprm['model'], prm['xyz'], prm['rpy'])
        pre_prm = prm
        rospy.set_param('/vscene/' + prm['frame'] + '/xyz', prm['xyz'])
        rospy.set_param('/vscene/' + prm['frame'] + '/rpy', prm['rpy'])
      else:
        result = False
        rospy.logerr("auto: model position format error")
    if not result:
      break
  return result

def model_solve(wait):
  result = False
  if set_clear():
    rospy.sleep(3)
    if recipe_load():
      retry_max = 1
      if 'solve_retry' in Param and Param['solve_retry']:
        retry_max = Param['solve_retry']
      rospy.loginfo("auto: solve retry max=%d", retry_max)
      update_campos()
      for prm in CamPos:
        result = False
        if 'type' in prm:
          if prm['type'] == 'move':
            result = robo_one_move(prm)
          else:
            result = robo_path_move(prm)
        if not result:
          break
      if result:
        rospy.loginfo("auto: all position capture finished")
        stats={}
        for num in range(retry_max):
          stats['retry'] = num+1
          pub_report.publish(str(stats))
          rospy.loginfo("auto: solve num=%d", num+1)
          result = set_solve()
          rospy.sleep(wait)
          if result: break
  return result

def update_user_pos(prm):
  if 'axis_frame_id' in prm and 'axis_base_frame_id' in prm and len(SolveTransform):
    cTw = getRT(Config['base_frame_id'], prm['axis_base_frame_id'])
    wTc = np.linalg.inv(cTw)
    cTc = tflib.toRT(tflib.dict2tf(SolveTransform))
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = prm['axis_base_frame_id']
    tf.child_frame_id = prm['axis_frame_id']
    tf.transform = tflib.fromRT(wTc.dot(cTc).dot(cTw))
    broadcaster.sendTransform(tf)
    print("auto: update ", prm['axis_frame_id'], " tf=", tf)
    stats={}
    stats['Cx'] = tf.transform.translation.x
    stats['Cy'] = tf.transform.translation.y
    stats['Cz'] = tf.transform.translation.z
    pub_report.publish(str(stats))

def get_solve_result(prm):
  global SolveTransform
  result = False
  if 'solve_save' in prm and prm['solve_save']:
    SolveTransform = {}
    if tf_lookup is not None:
      req = Config['base_frame_id'] + ' ' + Config['source_frame_id'] + ' ' + Config['target_frame_id']
      try:
        res = tf_lookup(req)
      except rospy.ServiceException as exc:
        print("Service did not process reqest: ", str(exc))
      else:
        result = True
        SolveTransform = json.loads(res.data)
        update_user_pos(prm)
    else:
      print("tf_lookup None")
  else:
    result = True
  return result

def model_seq_start():
  result = False
  wait = 1
  if 'solve_end_wait' in Param and Param['solve_end_wait']:
    wait = Param['solve_end_wait']
  if 'modelmove' in Param and Param['modelmove']:
    change_model_pos()
  set_clear()
  set_all_model_show(True, True)
  for prm in Model:
    result = False
    if 'model' in prm and 'recipe' in prm:
      rospy.loginfo("auto: model solve start model=%s", prm['model'])
      Param['recipe'] = prm['recipe']
      set_campos_change_data(prm)
      if model_solve(wait):
        get_solve_result(prm)
        set_model_end_show(prm)
        result = True
      else:
        break
    else:
      rospy.logerr("auto: model format error")
      break
  return result

def cb_seq_start(msg):
  if is_seq_exec():
    rospy.logerr("auto: already sequence exec")
  else:
    update_param()
    seq_start()
    retry_max = 1
    if 'seq_retry' in Param and Param['seq_retry']:
      retry_max = Param['seq_retry']
    if robo_init_move():
      rospy.sleep(1)
    for num in range(retry_max):
      rospy.loginfo("auto: model seq num=%d", num+1)
      if model_seq_start():
        rospy.loginfo("auto: model seq OK")
      else:
        rospy.logerr("auto: model seq NG")
        break
    seq_finished()

def cb_robo_moved(msg):
  rospy.loginfo("auto: cb_robo_moved")

def update_param():
  global Param, Model
  try:
    Param.update(rospy.get_param('~param'))
  except Exception as e:
    print("get_param exception:",e.args)
  Model = []
  for name in Param['seq_model']:
    item = get_model_param(name)
    Model.append(item)

def get_model_param(name):
  thispath = subprocess.getoutput("rospack find rsim_kmt")
  yamlpath = thispath + '/model.d/' + name + '/model.yaml'
  param = {}
  try:
    with open(yamlpath) as file:
      param = yaml.safe_load(file)
  except:
    rospy.logerr("auto: model yaml load error path=%s", yamlpath)
  return param

def set_campos_change_data(prm):
  global CamPos_Change
  CamPos_Change = {}
  if 'campos_change' in prm:
    CamPos_Change = prm['campos_change']

def update_campos():
  global CamPos
  prm = {}
  try:
    prm = rospy.get_param('/autoseq/campos')
  except Exception as e:
    print("campos get_param exception:",e.args)
  CamPos = []
  num = 0
  for key in prm.keys():
    if key.startswith('pos'):
      item = prm[key]
      if key in CamPos_Change:
        change = CamPos_Change[key]
        if 'path' in change:
          if 'xyz' in change['path']:
            item['path']['xyz'] = change['path']['xyz']
          if 'rpy' in change['path']:
            item['path']['rpy'] = change['path']['rpy']
      CamPos.append(item)
    elif key == 'num':
      num = prm['num']
  sorted(CamPos, key=lambda x: x['pos'])
  del CamPos[num:]

# Initialize the ROS Node
rospy.init_node('auto_seq', anonymous=True)
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print("config exception:",e.args)
###Load params
Param = {}
Model = []
CamPos = []
CamPos_Change = {}
SolveTransform = {}
update_param()
###Topics
pub_load = rospy.Publisher('~load', String, queue_size=1)
pub_rmove = rospy.Publisher('~rmove', Bool, queue_size=1)
pub_path_interval = rospy.Publisher('~path_interval', Bool, queue_size=1)
pub_path_point = rospy.Publisher('~path_point', Bool, queue_size=1)
pub_solve = rospy.Publisher('~solve', Bool, queue_size=1)
pub_clear = rospy.Publisher('~clear', Bool, queue_size=1)
sub_start = rospy.Subscriber('~start', Bool, cb_seq_start)
pub_mupdae = rospy.Publisher('~mupdate', Bool, queue_size=1)
pub_report = rospy.Publisher('/report', String, queue_size=1)
try:
  rospy.wait_for_service('/tf_lookup/query', 2000)
except rospy.ROSInterruptException as e:
  print("tf_lookup service not shutdown")
  tf_lookup = None
except rospy.ROSException as e:
  print("tf_lookup service not available")
  tf_lookup = None
else:
  tf_lookup = rospy.ServiceProxy('/tf_lookup/query', TextFilter)

#何故か下記がないとrospy.wait_for_messageで応答がない
sub_rmoved = rospy.Subscriber('~rmoved', Bool, cb_robo_moved)

seq_exec = False

####Bools
mTrue = Bool()
mTrue.data = True
mFalse = Bool()

tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

while not rospy.is_shutdown():
  time.sleep(0.01)

