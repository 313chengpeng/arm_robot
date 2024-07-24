#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def get_current_position():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_current_position_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 获取当前位姿
        current_pose = group.get_current_pose().pose

        rospy.loginfo("Current Pose:")
        rospy.loginfo("Position: x=%f, y=%f, z=%f" % (current_pose.position.x, current_pose.position.y, current_pose.position.z))
        rospy.loginfo("=========================================")
        rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f" % (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w))

        rospy.sleep(5)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        get_current_position()
    except rospy.ROSInterruptException:
        pass
