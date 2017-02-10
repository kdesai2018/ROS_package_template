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
import tf
import ar_track_alvar
from ar_track_alvar.msg import * 
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
    
    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(1)
    self.markerArray = MarkerArray()

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

  def arPoseMarkerCallback(self,msg):
    if(len(markers)>0):
      mark = msg.markers[0]

      p = [0]*7
      p[0] = mark.pose.pose.position.x
      p[1] = mark.pose.pose.position.y
      p[2] = mark.pose.pose.position.z
      p[3] = mark.pose.pose.orientation.x
      p[4] = mark.pose.pose.orientation.y
      p[5] = mark.pose.pose.orientation.z
      p[6] = mark.pose.pose.orientation.w

      print p


def main():
  tagTracker = TagTracking()

  ## ask if integrate object scene from code or not
  
    ##   Assigned tarPose the current Pose of the robot 
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  main()
