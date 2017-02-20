#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import *
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
from shape_msgs.msg import *
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians,degrees
import numpy

class CollisionObject:

  def __init__(self, planning_frame='linear_actuator_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Starting up")
    # rospy.init_node("tag_track")

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group = [moveit_commander.MoveGroupCommander("left_arm")] #change this to right_arm or left_arm

    self.planner = default_planner

    self.group[0].set_pose_reference_frame(planning_frame)

    self.continuous_joints = ['left_shoulder_pan_joint','left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous

    # self.publisher = rospy.Publisher('collision_object', CollisionObject)
    # self.publisher = rospy.Publisher('planning_scene', PlanningScene)
    self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
    self.markerArray = MarkerArray()
    rospy.sleep(2)
    

  def get_FK(self, root = 'linear_actuator_link'):

    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

    header = std_msgs.msg.Header()
    header.frame_id = root
    header.stamp = rospy.Time.now()
    fk_link_names = ['left_ee_link']
    robot_state = self.robot.get_current_state()    
    try:
      reply=compute_fk(header,fk_link_names,robot_state)
      return reply.pose_stamped

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  
  def publish_point(self, pose,scale,color):
    marker = Marker()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g= color[1]
    marker.color.b = color[2]
    marker.pose = pose
    marker.header.frame_id = "/linear_actuator_link"
    # print self.marker
    # markerArray = MarkerArray()
    self.markerArray.markers.append(marker)

    id = 0
    for m in self.markerArray.markers:
      m.id = id
      id += 1
    # print self.markerArray
    self.publisher.publish(self.markerArray)

  def publish_collision_object(self):
    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "linear_actuator_link"
    pose.pose.position.x = 0.3;
    pose.pose.position.y = -0.15;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    scale = [.4,.4,.6]
    self.scene.add_box("right_arm",pose,scale)
    rospy.sleep(4)


    marker_pos = geometry_msgs.msg.Pose()
    marker_pos.position.x = pose.pose.position.x
    marker_pos.position.y = pose.pose.position.y
    marker_pos.position.z = pose.pose.position.z
    marker_pos.orientation.x = 0
    marker_pos.orientation.y = 0
    marker_pos.orientation.z = 0
    marker_pos.orientation.w = 1
    self.publish_point(marker_pos,scale,[1,0,0])

    pose.header.frame_id = "base_link"
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    scale = [0.62,1.23,1]
    self.scene.add_box("table",pose,scale)
    rospy.sleep(4)

    marker_pos = geometry_msgs.msg.Pose()
    marker_pos.position.x = pose.pose.position.x
    marker_pos.position.y = pose.pose.position.y
    marker_pos.position.z = pose.pose.position.z
    marker_pos.orientation.x = 0
    marker_pos.orientation.y = 0
    marker_pos.orientation.z = 0
    marker_pos.orientation.w = 1
    self.publish_point(marker_pos,scale,[0,1,0])

  

  def getRobotCurrentState(self):
    return self.group[0].get_current_pose().pose
    

  
    
def main():
  collisionObject = CollisionObject()
  counter = 0
  while (not(rospy.is_shutdown())):
    # print tagTracker.get_FK()[0].pose
    collisionObject.publish_collision_object()

    print "done" + str(counter)
    counter+=1
    rospy.sleep(1)
  ## ask if integrate object scene from code or not
  
    ##   Assigned tarPose the current Pose of the robot 
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  main()
  rospy.spin()
