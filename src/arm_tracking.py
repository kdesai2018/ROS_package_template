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
import operator

import requests
import tf2_ros
import tf
from ar_track_alvar_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from std_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians
import numpy

class TagTracking:

  def __init__(self, planning_frame='linear_actuator_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Starting up")
    # rospy.init_node("tag_track")
    try:
      rospy.wait_for_service('compute_ik')
    except rospy.ROSExecption, e:
      rospy.logerr("No moveit service detected. Exiting")
      exit()
    else:
      rospy.loginfo("MoveIt detected: arm planner loading")
    # self.pose = geometry_msgs.msg.PoseStamped()
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute   motions on the left
    ## arm.
    self.group = [moveit_commander.MoveGroupCommander("left_arm")] #change this to right_arm or left_arm

    # Set the planner
    self.planner = default_planner

    # Set the planning pose reference frame
    self.group[0].set_pose_reference_frame(planning_frame)
    # self.group[1].set_pose_reference_frame(planning_frame)
    # Set continuous joint names
    self.continuous_joints = ['right_shoulder_pan_joint','right_wrist_1_joint','right_wrist_2_joint','right_wrist_3_joint']
    # NOTE: order that moveit currently is configured
    # ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous

    self.kinect_angle_pub = rospy.Publisher('/tilt_controller/command',Float64)
    rospy.sleep(1)
    self.kinect_angle = Float64()
    self.kinect_angle.data = 0.3925
    self.kinect_angle_pub.publish(self.kinect_angle)

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arPoseMarkerCallback)
    self.currentMarkerPose = [0]*3
    self.goalMarkerPose = [1,0,0] #to be determined before running experiment
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(2)
    trans = tfBuffer.lookup_transform("kinect_link","linear_actuator_link",rospy.Time(0))
    self.trans = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
    # print trans

    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(1)
    self.markerArray = MarkerArray()

  def _simplify_angle(self, angle):
    # Very simple function that makes sure the angles are between -pi and pi
    if angle > pi:
      while angle > pi:
        angle -= 2*pi
    elif angle < -pi:
      while angle < -pi:
        angle += 2*pi

    return angle

  def _simplify_joints(self, joint_dict):

    if isinstance(joint_dict, dict):
      simplified_joints = dict()
      for joint in joint_dict:
        # Pull out the name of the joint
        joint_name = '_'.join(joint.split('_')[1::])
        if joint_name in self.continuous_joints:
          simplified_joints[joint] = self._simplify_angle(joint_dict[joint])
        else:
          simplified_joints[joint] = joint_dict[joint]
    elif isinstance(joint_dict, list):
      simplified_joints = []
      for i in xrange(len(joint_dict)):
        
        a = joint_dict[i]

        if i in self.continuous_joints_list:
          simplified_joints.append(self._simplify_angle(a))
        else:
          simplified_joints.append(a)
    return simplified_joints

#   '''Older functions - left for backwards compatibility'''

  def plan_jointTargetInput(self,target_joint,n):
    ## input: target joint angles (list) of the robot
    ## output: plan from current joint angles to the target one
    try:
      self.group[n].set_joint_value_target(self._simplify_joints(target_joint))
      self.group[n].set_planner_id(self.planner)
      planAns=self.group[n].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  def get_IK(self, newPose,n, root = None):
    ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

    wkPose = geometry_msgs.msg.PoseStamped()
    if root is None:
      wkPose.header.frame_id = 'linear_actuator_link' # name:odom
    else:
      wkPose.header.frame_id = root

    wkPose.header.stamp=rospy.Time.now()
    wkPose.pose=newPose

    msgs_request = moveit_msgs.msg.PositionIKRequest()
    msgs_request.group_name = 'left_arm'; # takes in left_arm as argument at all times.
    # msgs_request.ik_link_name = self.group[n].get_name()
    # print self.group[n].get_name()
    msgs_request.robot_state = self.robot.get_current_state()
    msgs_request.pose_stamped = wkPose
    msgs_request.timeout.secs = 2
    msgs_request.avoid_collisions = False
    # print msgs_request
    try:
      jointAngle=compute_ik(msgs_request)
      ans=list(jointAngle.solution.joint_state.position[1:7])
      if jointAngle.error_code.val == -31:
        print 'No IK solution'
        return None
      return ans

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def arPoseMarkerCallback(self,msg):
    if(len(msg.markers)>0):
      mark = msg.markers[0]
      p = [0]*3
      p[0] = mark.pose.pose.position.x #width
      p[1] = mark.pose.pose.position.y #height
      p[2] = mark.pose.pose.position.z #depth
      self.currentMarkerPose = p


  def diffEE_AR(self,ee_pos,ar_pos):
    return map(operator.sub,ee_pos, ar_pos)

  def printMarkerPose(self):
    self.publish_point(self.markerPose[2]+self.trans[0],self.markerPose[0]+self.trans[1],self.markerPose[1]-self.trans[2])
    
  def getInitMarkerPose(self):
      print "Initial Marker Pose is:"
      print self.currentMarkerPose
      self.publish_point(self.currentMarkerPose[2]+self.trans[0],self.currentMarkerPose[0]+self.trans[1],self.currentMarkerPose[1]-self.trans[2])
      return self.currentMarkerPose
      
  def quat_to_rpy(self,q):
      rpy = [0, 0, 0]
      PyKDL.Quaternion(q[0], q[1], q[2], q[3]).GetRPY(rpy[0], rpy[1], rpy[2])
      return rpy
  def getRobotCurrentState(self):
    pos = self.group[0].get_current_pose().pose
    state = [0]*3
    state[0] = pos.position.x
    state[1] = pos.position.y
    state[2] = pos.position.z
    # orientation = self.quat_to_rpy([pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w])
    # state[3] = orientation[0]
    # state[4] = orientation[1]
    # state[5] = orientation[2]
    # print "robot state"
    # print state
    return state

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
  if (not(rospy.is_shutdown())):
    # tagTracker.printMarkerPose()
    init_pos = tagTracker.getInitMarkerPose()
    robot_pos = tagTracker.getRobotCurrentState()
    diff = tagTracker.diffEE_AR(robot_pos,init_pos)
    tarPose = geometry_msgs.msg.Pose()

    tarPose.orientation = tagTracker.group[0].get_current_pose().pose.orientation
    position = map(operator.add,tagTracker.goalMarkerPose,diff)
    tarPose.position.x = position[0]
    tarPose.position.y = position[1]
    tarPose.position.z = position[2]
    jointTarg = tagTracker.get_IK(tarPose,0)
    planTraj = tagTracker.plan_jointTargetInput(jointTarg,0)
    if(planTraj!=None):
          tagTracker.group[0].execute(planTraj)
    rospy.sleep(0.2)
  ## ask if integrate object scene from code or not
  
    ##   Assigned tarPose the current Pose of the robot 
       
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  main()
  rospy.spin()
