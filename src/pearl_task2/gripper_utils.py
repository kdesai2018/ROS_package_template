#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from geometry_msgs.msg import *
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import time
import operator
from vector_msgs.msg import JacoCartesianVelocityCmd, LinearActuatorCmd, GripperCmd, GripperStat
import requests
import tf2_ros
import tf2_geometry_msgs
import tf
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians,degrees
import numpy

class Gripper:
  def __init__(self, prefix='right'):
    self.pub_grp  = rospy.Publisher('/vector/'+prefix+'_gripper/cmd', GripperCmd, queue_size = 10)
    self.cmd = GripperCmd()
    
    #i have it here but it is not useful
    #rospy.Subscriber('/vector/right_gripper/joint_states', JointState, self.js_cb)
    #self.last_js_update = None
    #self.joint_state = None
    
    rospy.Subscriber('/vector/'+prefix+'_gripper/stat', GripperStat, self.st_cb)
    self.last_st_update = None
    self.gripper_stat = GripperStat()
    

  #def js_cb(self, inState):
  #  self.joint_state = inState.position  
  #  self.last_js_update = rospy.get_time()
    
  def st_cb(self, inStat):
    self.gripperStat = inStat
    self.last_st_update = None
    
  def is_ready(self):
    return self.gripperStat.is_ready
  
  def is_reset(self):
    return self.gripperStat.is_reset

  def is_moving(self):
    return self.gripperStat.is_moving
    
  def object_detected(self):
    return self.gripperStat.obj_detected
    
  def get_pos(self):
    return self.gripperStat.position
    
  def get_commanded_pos(self):
    return self.gripperStat.requested_position

  def get_applied_current(self):
    return self.gripperStat.current

  def set_pos(self, position, iterations,speed = 0.02, force = 100, rate = 10):
    print iterations
    self.cmd.position = position
    self.cmd.speed = speed
    self.cmd.force = force
    rrate = rospy.Rate(rate)
    for i in range(0,iterations):
      self.pub_grp.publish(self.cmd)
      rrate.sleep()  
    
  def open(self, force,speed = 0.02):
    self.set_pos(0.085,5,speed,force)
    
  def close(self, force,speed = 0.02):
    print force
    self.set_pos(0,10,speed,force)



    
def main():
  gripper = Gripper()
  counter = 0
  while (not(rospy.is_shutdown())):

    open = input("open = 1, close =0")
    forceInp = input("force?")
    if(open==-1 or forceInp==-1):
	    return
    if(open==0):
   	    gripper.close(forceInp)
    else:
	    gripper.open(100)
   
    print "done" + str(counter)
    counter+=1
    rospy.sleep(1)
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  #moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('gripperutils', anonymous=True)
  main()
  rospy.spin()

