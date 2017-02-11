#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import time
import requests
import tf2_ros
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians

class TagTracking:

  def __init__(self, planning_frame='kinect_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Starting up")
    rospy.init_node("tag_track")

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arPoseMarkerCallback)
    self.markerPose = [0]*3
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(2)
    trans = tfBuffer.lookup_transform("kinect_link","linear_actuator_link",rospy.Time(0))
    self.trans = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
    print trans

    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(1)
    self.markerArray = MarkerArray()

  def arPoseMarkerCallback(self,msg):
    if(len(msg.markers)>0):
      mark = msg.markers[0]
      p = [0]*3
      p[0] = mark.pose.pose.position.x #width
      p[1] = mark.pose.pose.position.y #height
      p[2] = mark.pose.pose.position.z #depth
      self.markerPose = p

  def printMarkerPose(self):
    self.publish_point(self.markerPose[2]+self.trans[0],self.markerPose[0]+self.trans[1],self.markerPose[1]+self.trans[2])
    pass

  def publish_point(self, x,y,z):
    marker = Marker()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.header.frame_id = "/linear_actuator_link"
    # print self.marker
    # self.markerArray = MarkerArray()
    self.markerArray.markers.append(marker)

    id = 0
    for m in self.markerArray.markers:
      m.id = id
      id += 1
    # print self.markerArray
    self.publisher.publish(self.markerArray)
    
def main():
  tagTracker = TagTracking()
  while (not(rospy.is_shutdown())):
    tagTracker.printMarkerPose()
    rospy.sleep(0.2)
  ## ask if integrate object scene from code or not
  
    ##   Assigned tarPose the current Pose of the robot 
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  main()
  rospy.spin()
