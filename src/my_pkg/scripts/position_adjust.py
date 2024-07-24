# #!/usr/bin/env python

# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

# def move_to_pose(group, pose):
#     group.set_pose_target(pose)
#     plan = group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()
#     return plan

# def main():
#     # 初始化ROS节点
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('move_rm65_b', anonymous=True)

#     # 初始化机器人组
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group_name = "arm"  # 根据实际配置修改
#     group = moveit_commander.MoveGroupCommander(group_name)
    
#     display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#     # 设置目标位置A
#     pose_A = geometry_msgs.msg.Pose()
#     pose_A.position.x = 1.0
#     pose_A.position.y = 0.4
#     pose_A.position.z = 0.1
#     pose_A.orientation.w = 0.4

#     # 移动到位置A
#     move_to_pose(group, pose_A)

#     rospy.sleep(2)

#     # 设置目标位置B
#     pose_B = geometry_msgs.msg.Pose()
#     pose_B.position.x = 1.0
#     pose_B.position.y = 0.6
#     pose_B.position.z = 0.1
#     pose_B.orientation.w = 0.4

#     # 移动到位置B
#     move_to_pose(group, pose_B)

#     rospy.sleep(2)

#     # # 设置目标位置C
#     # pose_C = geometry_msgs.msg.Pose()
#     # pose_C.position.x = 1.0
#     # pose_C.position.y = 0.4
#     # pose_C.position.z = 0.5
#     # pose_C.orientation.w = 0.4

#     # move_to_pose(group, pose_C)


#     # 关闭并退出
#     moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python

# import rospy
# import sys
# import moveit_commander
# from geometry_msgs.msg import Pose

# def move_arm():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('move_arm_node', anonymous=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group = moveit_commander.MoveGroupCommander("arm")

#     # 设置目标位置
#     pose_target = Pose()
#     pose_target.orientation.w = 1.0
#     pose_target.position.x = 0.4
#     pose_target.position.y = 0.1
#     pose_target.position.z = 0.4
#     group.set_pose_target(pose_target)

#     # 规划路径并执行
#     plan = group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()

#     moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     try:
#         move_arm()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
import moveit_msgs.msg

# def move_to_joint_positions(group, joint_goal):
#     group.go(joint_goal, wait=True)
#     group.stop()
#     group.clear_pose_targets()

# def move_to_pose_target(group, pose_goal):
#     group.set_pose_target(pose_goal)
#     plan = group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()
#     return plan

# def move_linear(group, pose_goal):
#     waypoints = []
#     waypoints.append(group.get_current_pose().pose)
#     waypoints.append(pose_goal)

#     (plan, fraction) = group.compute_cartesian_path(
#                             waypoints,   # 路点列表
#                             0.01,        # eef_step
#                             0.0)         # jump_threshold
#     plan = group.go(wait=True)
#     if plan:
#         group.execute(plan, wait=True)
        
#     else:
#         rospy.logerr("Cartesian path planning failed")

# def is_pose_reachable(group, pose):
#     group.set_pose_target(pose)
#     plan = group.plan()
#     group.clear_pose_targets()
#     return plan[0] is not None

# def main():
#     try:
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_robot', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "arm"  # 根据实际配置修改
#         group = MoveGroupCommander(group_name)
        
#         display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#         # 设置规划时间和重试次数
#         group.set_planning_time(2)  # 设置更长的规划时间
#         group.allow_replanning(True)  # 允许重新规划

#         # MOVEJ: 移动到目标关节位置
#         joint_goal = group.get_current_joint_values()
#         joint_goal[0] = 2.5
#         joint_goal[1] = 1.0
#         joint_goal[2] = 1.15
#         joint_goal[3] = 0.0
#         joint_goal[4] = 1.0
#         joint_goal[5] = 0

#         rospy.loginfo("Executing MOVEJ")
#         move_to_joint_positions(group, joint_goal)

#         rospy.sleep(2)


#         # MOVEL: 笛卡尔空间直线运动
#         linear_goal = Pose()
#         linear_goal.position.x = 0.6
#         linear_goal.position.y = 0.0
#         linear_goal.position.z = 0.5
#         linear_goal.orientation.w = 1.0

#         if not is_pose_reachable(group, linear_goal):
#             rospy.logerr("The pose is not reachable")
#             return

#         rospy.loginfo("Executing MOVEL")
#         move_linear(group, linear_goal)

#         rospy.sleep(2)

#         moveit_commander.roscpp_shutdown()

#     except rospy.ROSInterruptException:
#         pass

# if __name__ == '__main__':
#     main()



def move_to_joint_positions(group, joint_goal):
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()

def move_to_pose_target(group, pose_goal):
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return plan

def move_linear(group, waypoints):
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,   # 路点列表
                            0.01,        # eef_step
                            0.0)         # jump_threshold

    if plan:
        group.execute(plan, wait=True)
    else:
        rospy.logerr("Cartesian path planning failed")

def is_pose_reachable(group, pose):
    group.set_pose_target(pose)
    plan = group.plan()
    group.clear_pose_targets()
    return plan[0] is not None


def main():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_robot', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"  # 根据实际配置修改
        group = MoveGroupCommander(group_name)
        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # 设置规划时间和重试次数
        group.set_planning_time(10)  # 设置更长的规划时间
        group.allow_replanning(True)  # 允许重新规划

        # MOVEJ: 移动到目标关节位置
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = -0.256
        joint_goal[2] = -0.956
        joint_goal[3] = 0.0
        joint_goal[4] = -1.679
        joint_goal[5] = 0

        rospy.loginfo("Executing MOVEJ")
        move_to_joint_positions(group, joint_goal)

        rospy.sleep(2)

        # 获取末端执行机构的位置
        current_pose = group.get_current_pose().pose
        rospy.loginfo("Current Pose: %s" % current_pose)

        while not rospy.is_shutdown():
            # 设置水平向左移动到最远位置
            
            max_left_pose = Pose()
            max_left_pose.position.x = current_pose.position.x 
            max_left_pose.position.y = current_pose.position.y+0.2 # 假设0.2是可以移动的最大距离
            max_left_pose.position.z = current_pose.position.z
            max_left_pose.orientation = current_pose.orientation
            

            
            if not is_pose_reachable(group, max_left_pose):
                rospy.logerr("The pose is not reachable")
                return

            rospy.loginfo("Moving to the max left position")
            waypoints = [current_pose, max_left_pose]
            move_linear(group, waypoints)
            
            # 打印移动后的位置信息
            current_pose_2 = group.get_current_pose().pose
            rospy.loginfo("Pose after moving left: %s" % current_pose_2)
            


            rospy.sleep(1)

            # 返回到初始位置
            rospy.loginfo("Returning to the initial position")
            waypoints = [max_left_pose, current_pose]
            move_linear(group, waypoints)
            
            

            rospy.sleep(1)

            # moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
