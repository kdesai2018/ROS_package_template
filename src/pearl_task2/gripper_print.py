#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import String
from robotiq_85_msgs.msg import GripperStat

class GripperPrint:
    def __init__(self):
        self.pub_grp  = rospy.Publisher('/gripper_print', String, queue_size = 10)
        
        rospy.Subscriber('/gripper/stat', GripperStat, self.callback)

        rospy.sleep(2)

    def callback(self, msg):
        self.pub_grp.publish(String(str(msg.position)))

def main():
  gripper = GripperPrint()
  
if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.  
  #moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('gripper_print', anonymous=True)
  main()
  rospy.spin()

