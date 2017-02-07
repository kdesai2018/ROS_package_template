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
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import pi, floor, ceil, fabs, sin, cos, radians

class ArmMoveIt:

  def __init__(self, planning_frame='linear_actuator_link', default_planner="RRTConnectkConfigDefault"):
    # r = requests.get("http://10.5.5.9/gp/gpControl/command/mode?p=1")
    # Make sure the moveit service is up and running
    rospy.logwarn("Waiting for MoveIt! to load")
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
    self.group = [moveit_commander.MoveGroupCommander("right_arm")]

    # Set the planner
    self.planner = default_planner

    # Set the planning pose reference frame
    self.group[0].set_pose_reference_frame(planning_frame)
    # self.group[1].set_pose_reference_frame(planning_frame)
    # Set continuous joint names
    self.continuous_joints = ['left_shoulder_pan_joint','left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint',
    'right_shoulder_pan_joint','right_wrist_1_joint','right_wrist_2_joint','right_wrist_3_joint']
    # NOTE: order that moveit currently is configured
    # ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5] # joints that are continous
    topic = 'visualization_marker_array'
    self.publisher = rospy.Publisher(topic, MarkerArray)
    rospy.sleep(1)
    self.markerArray = MarkerArray()


    

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
    msgs_request.group_name = self.group[n].get_name() # name: arm
    # msgs_request.robot_state = robot.get_current_state()
    msgs_request.pose_stamped = wkPose
    msgs_request.timeout.secs = 2
    msgs_request.avoid_collisions = False

    try:
      jointAngle=compute_ik(msgs_request)
      ans=list(jointAngle.solution.joint_state.position[1:7])
      if jointAngle.error_code.val == -31:
        print 'No IK solution'
        return None
      return ans

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e


  def set_robot_state_pose(self, traj):
    '''Gets the current robot state pose and sets it to the joint pose'''
    cur_robot_state = self.robot.get_current_state()
    last_point = traj.points[-1].positions
    # convert the joints to array
    joints = [x for x in cur_robot_state.joint_state.position]
    for i in xrange(len(traj.joint_names)):
      # Find index of joint
      joint_name = traj.joint_names[i]
      idx = cur_robot_state.joint_state.name.index(joint_name)
      joints[idx] = last_point[i]

    # Set full joint tuple now
    cur_robot_state.joint_state.position = joints

    return cur_robot_state


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

  def calc_orientation(self,angle):
    quaternion = tf.transformations.quaternion_from_euler(0, radians(90), -radians(angle+90))
    pose= geometry_msgs.msg.Pose()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]              
    return  pose.orientation                

  def calc_mov(self,angle,radius,center,z):
    rad = radians(angle)
    x = center+radius*(sin(rad))
    y = 1.0*radius*(cos(rad))
    poseTmp= geometry_msgs.msg.Pose()
    poseTmp.position.x=x
    poseTmp.position.y=y
    poseTmp.position.z=z
    
    return poseTmp.position


  def execute_circle(self,jump,radius,center,z,arm):

    tarPose = geometry_msgs.msg.Pose()

    for angle in range(-40,-39,jump):
        tarPose.position = self.calc_mov(angle,radius,center,z)
        tarPose.orientation = self.calc_orientation(angle)
        self.publish_point(tarPose.position.x,tarPose.position.y,tarPose.position.z)
        print '\n The target coordinate is: %s \n' %tarPose 
        jointTarg = self.get_IK(tarPose,arm)
        planTraj = self.plan_jointTargetInput(jointTarg,arm)
        if(planTraj!=None):
          print "going to angle " + str(angle)   
          self.group[arm].execute(planTraj)
          print str(arm) + "arm"
          time.sleep(5)
          # r = requests.get("http://10.5.5.9/gp/gpControl/command/shutter?p=1")  


  def auto_circle(self,rad_outer,rad_inner,center):


    self.publish_point(center,0,1.2 )
    jump = 30 #hard coded for now
    self.execute_circle(jump,rad_outer,center,-.15,0)


def main():
  arm = ArmMoveIt()

  tarPose = geometry_msgs.msg.Pose()

  ## ask if integrate object scene from code or not
  
  if not rospy.is_shutdown():
    
    ##   Assigned tarPose the current Pose of the robot 
    arm.auto_circle(0.37,0.2,0.7)
   
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('vector_basic_IK', anonymous=True)
  
  main()
