#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rm_msgs.msg import MoveL, MoveJ_P, Plan_State

# Global variable to prevent MoveL from executing repeatedly
flag = 0

def plan_state_callback(msg):
    # global flag
    # # Print the received message to show whether the arm has completed its movement
    if msg.state:
    #     if flag == 0:
    #         rospy.loginfo("The first trajectory has been executed")
    #         rospy.loginfo("Prepare to execute Instruction MoveL")

    #         # Delay for 2 seconds to ensure the arm is stable after completing the previous trajectory
    #         rospy.sleep(2.0)

    #         # Control the arm to move linearly using MoveL command
    #         # Define a target pose for the MoveL command (adjust the Z coordinate of the initial pose) to make the end effector move vertically upwards
    #         moveL_target_pose = MoveL()
    #         moveL_target_pose.Pose.position.x = -0.0245299
    #         moveL_target_pose.Pose.position.y = 0.33842098
    #         moveL_target_pose.Pose.position.z = 0.295527
    #         moveL_target_pose.Pose.orientation.x = 0.6501861
    #         moveL_target_pose.Pose.orientation.y = -0.759086
    #         moveL_target_pose.Pose.orientation.z = -0.031065
    #         moveL_target_pose.Pose.orientation.w = -0.008943
    #         moveL_target_pose.speed = 0.3

    #         # Publish the pose
    #         moveL_pub.publish(moveL_target_pose)

    #         flag = 1
    #     elif flag == 1:
        rospy.loginfo("MoveL has been executed")
        flag = 2
    else:
        rospy.loginfo("*******Plan State Fail")

if __name__ == '__main__':
    rospy.init_node('api_moveL_demo', anonymous=True)

    """
    1. Relevant Initialization
    """
    # Space planning command Publisher
    moveJ_P_pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)

    # Linear planning command Publisher
    moveL_pub = rospy.Publisher('/rm_driver/MoveL_Cmd', MoveL, queue_size=10)

    # Subscribe to the robot arm execution state topic
    plan_state_sub = rospy.Subscriber('/rm_driver/Plan_State', Plan_State, plan_state_callback)

    rospy.loginfo("Waiting for the robot to be ready...")
    rospy.sleep(2.0)

    """
    2. Control the robot arm to move to the initial pose using the MoveJ_P command
    """
    # Define a target pose for the MoveJ_P command
    moveJ_P_target_pose = MoveJ_P()
    moveJ_P_target_pose.Pose.position.x = -0.0245299
    moveJ_P_target_pose.Pose.position.y = 0.33842098
    moveJ_P_target_pose.Pose.position.z = 0.315527
    moveJ_P_target_pose.Pose.orientation.x = 0.6501861
    moveJ_P_target_pose.Pose.orientation.y = -0.759086
    moveJ_P_target_pose.Pose.orientation.z = -0.031065
    moveJ_P_target_pose.Pose.orientation.w = -0.008943
    moveJ_P_target_pose.speed = 0.3

    # Publish the space planning command to move the arm to the initial pose
    moveJ_P_pub.publish(moveJ_P_target_pose)
    moveL_target_pose = MoveL()
    moveL_target_pose.Pose.position.x = -0.0245299
    moveL_target_pose.Pose.position.y = 0.33842098
    moveL_target_pose.Pose.position.z = 0.295527
    moveL_target_pose.Pose.orientation.x = 0.6501861
    moveL_target_pose.Pose.orientation.y = -0.759086
    moveL_target_pose.Pose.orientation.z = -0.031065
    moveL_target_pose.Pose.orientation.w = -0.008943
    moveL_target_pose.speed = 0.3
    moveL_pub.publish(moveL_target_pose)
    rospy.sleep(2)
    

    moveL_target_pose_2 = MoveL()
    moveL_target_pose_2.Pose.position.x = -0.1245299
    moveL_target_pose_2.Pose.position.y = 0.33842098
    moveL_target_pose_2.Pose.position.z = 0.295527
    moveL_target_pose_2.Pose.orientation.x = 0.6501861
    moveL_target_pose_2.Pose.orientation.y = -0.759086
    moveL_target_pose_2.Pose.orientation.z = -0.031065
    moveL_target_pose_2.Pose.orientation.w = -0.008943
    moveL_target_pose_2.speed = 0.5
    
    moveL_target_pose_3 = MoveL()
    moveL_target_pose_3.Pose.position.x = 0.1745299
    moveL_target_pose_3.Pose.position.y = 0.33842098
    moveL_target_pose_3.Pose.position.z = 0.295527
    moveL_target_pose_3.Pose.orientation.x = 0.6501861
    moveL_target_pose_3.Pose.orientation.y = -0.759086
    moveL_target_pose_3.Pose.orientation.z = -0.031065
    moveL_target_pose_3.Pose.orientation.w = -0.008943
    moveL_target_pose_3.speed = 0.3

    
    while not rospy.is_shutdown():

        moveL_pub.publish(moveL_target_pose_2)
        rospy.sleep(5)

        moveL_pub.publish(moveL_target_pose_3)
        rospy.sleep(5)
        
           

    # Keep the node running and process callbacks as they come
    rospy.spin()
