# import rospy
# from std_msgs.msg import Float32
# import moveit_commander
# from geometry_msgs.msg import Pose
# import sys
# import moveit_msgs.msg


# target_distance = 0.10  # 目标距离为10 cm
# tolerance = 0.005  # 容差为0.5 cm

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

# def move_linear(group, waypoints):
#     (plan, fraction) = group.compute_cartesian_path(
#                             waypoints,   # 路点列表
#                             0.01,        # eef_step
#                             0.0)         # jump_threshold

#     if plan:
#         group.execute(plan, wait=True)
#     else:
#         rospy.logerr("Cartesian path planning failed")

# def is_pose_reachable(group, pose):
#     group.set_pose_target(pose)
#     plan = group.plan()
#     group.clear_pose_targets()
#     return plan[0] is not None

# def adjust_height(group, target_height):
#     current_pose = group.get_current_pose().pose
#     current_height = current_pose.position.z

#     if target_height > target_distance + tolerance:
#         new_height = current_height - 0.05  # 下调
#         rospy.loginfo(f"下调到 {new_height} m")
#     elif target_height < target_distance - tolerance:
#         new_height = current_height + 0.05  # 上调
#         rospy.loginfo(f"上调到 {new_height} m")
#     else:
#         new_height = current_height  # 保持高度
#         rospy.loginfo(f"保持高度 {new_height} m")

#     current_pose.position.z = new_height
#     move_to_pose_target(group, current_pose)

# def callback(data):
#     target_height = data.data
#     adjust_height(group, target_height)
#     move_back_and_forth(group)

# def move_back_and_forth(group):
#     current_pose = group.get_current_pose().pose

#     max_left_pose = Pose()
#     max_left_pose.position.x = current_pose.position.x
#     max_left_pose.position.y = current_pose.position.y + 0.2
#     max_left_pose.position.z = current_pose.position.z
#     max_left_pose.orientation = current_pose.orientation
    
#     if not is_pose_reachable(group, max_left_pose):
#         rospy.logerr("The pose is not reachable")
#         return

#     rospy.loginfo("Moving to the max left position")
#     waypoints = [current_pose, max_left_pose]
#     move_linear(group, waypoints)
#     rospy.sleep(2)

#     rospy.loginfo("Returning to the initial position")
#     waypoints = [max_left_pose, current_pose]
#     move_linear(group, waypoints)
#     rospy.sleep(2)

# def arm_control():
#     global group

#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('arm_control', anonymous=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group_name = "arm"  # 根据实际配置修改
#     group = moveit_commander.MoveGroupCommander(group_name)
    
#     display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#     group.set_planning_time(10)
#     group.allow_replanning(True)

#     joint_goal = group.get_current_joint_values()
#     joint_goal[0] = 0
#     joint_goal[1] = -0.256
#     joint_goal[2] = -0.956
#     joint_goal[3] = 0.0
#     joint_goal[4] = -1.679
#     joint_goal[5] = 0

#     rospy.loginfo("Executing MOVEJ")
#     move_to_joint_positions(group, joint_goal)

#     rospy.sleep(2)

#     rospy.Subscriber('distance', Float32, callback)

#     rospy.spin()

#     moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     try:
#         arm_control()
#     except rospy.ROSInterruptException:
#         pass

# V2

import rospy
from std_msgs.msg import Float32
import moveit_commander
from geometry_msgs.msg import Pose
import sys
import moveit_msgs.msg
from rm_msgs.msg import MoveL, MoveJ, MoveJ_P


target_height = 130  # 目标距离为120 mm
tolerance = 20  # 容差为20 mm

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
        # print('plan:', plan)
        print('fraction:', fraction)
        group.execute(plan, wait=True)
    else:
        rospy.logerr("无法规划笛卡尔路径")

def is_pose_reachable(group, pose):
    group.set_pose_target(pose)
    plan = group.plan()
    group.clear_pose_targets()
    return plan[0] is not None

def adjust_height(group, current_height_sensor):
    global current_z    
    print('当前高度：', current_height_sensor)

    current_pose = group.get_current_pose().pose    # 机械臂内部坐标的位置
    current_height = current_pose.position.z # 用于调整机械臂的位置

    
    if current_height_sensor > target_height + tolerance:
        
        current_z = current_height - 0.05  # 下调
        group.stop()    # 停止机械臂
        rospy.loginfo('即将下调')
        current_pose.position.z = current_z
        move_to_pose_target(group, current_pose)
        if not move_to_pose_target(group, current_pose):
            rospy.logerr('高度调整失败位置')
        rospy.sleep(5)
        rospy.loginfo('高度调整完成')

    elif current_height_sensor < target_height - tolerance:
        group.stop()    # 停止机械臂
        current_z = current_height + 0.05  # 上调
        rospy.loginfo('即将上调')
        move_to_pose_target(group, current_pose)
        # exec_flag = group.execute(plan, wait=True)
        if not move_to_pose_target(group, current_pose):
            rospy.logerr('高度调整失败位置')
        rospy.sleep(5)
        rospy.loginfo('高度调整完成')

    else:
        current_z = current_height  # 保持高度
        rospy.loginfo('保持高度')
        



def callback(data):
    current_height_sensor = data.data # 获取传感器的数据
    adjust_height(group, current_height_sensor)

def move_back_and_forth(group):
    global current_z 
    start_pose = group.get_current_pose().pose
    rospy.loginfo(f"开始时的z坐标: {current_z}")
    start_pose.position.z = current_z
    
    
    rospy.loginfo("向point2移动")
    end_pose = Pose()
    end_pose.position.x = start_pose.position.x -0.05
    end_pose.position.y = start_pose.position.y + 0.05
    end_pose.position.z = current_z
    end_pose.orientation = start_pose.orientation
    

    while not rospy.is_shutdown():
        # Move to end pose
        start_pose.position.z = current_z
        end_pose.position.z = current_z
        if not is_pose_reachable(group, end_pose):
            rospy.logerr("终点路径不可达")
            return
        else:
            rospy.loginfo("向point2移动")
            waypoints = [start_pose, end_pose]
            move_linear(group, waypoints)
            rospy.sleep(5)
            # my_pose = group.get_current_pose().pose
            # rospy.loginfo(f"my_pose position END: {my_pose}")
            if not move_linear(group, waypoints):
                rospy.logerr("移动到终点失败")
                now_pose = group.get_current_pose().pose
                rospy.loginfo(f"失败 position: {now_pose}")
            else:
                rospy.loginfo("移动到终点成功")
                now_pose = group.get_current_pose().pose
        rospy.sleep(2)
        if not is_pose_reachable(group, end_pose):
            rospy.logerr("The start pose is not reachable")
            return
        # Move back to start pose
        else:
            rospy.loginfo("Returning to the start position")
            waypoints = [end_pose, start_pose]
            move_linear(group, waypoints)
            rospy.sleep(5)
            if not move_linear(group, waypoints):
                rospy.logerr("移动到起点失败")
                now_pose = group.get_current_pose().pose
                rospy.loginfo(f"失败 position: {now_pose}")
            else:
                rospy.loginfo("移动到起点成功")
                now_pose = group.get_current_pose().pose
            
        rospy.sleep(2)

def arm_control():
    global group, current_z

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"  # 根据实际配置修改
    group = moveit_commander.MoveGroupCommander(group_name)
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    group.set_planning_time(10)
    group.allow_replanning(True)

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
    current_pose = group.get_current_pose().pose
    current_z = current_pose.position.z

    rospy.Subscriber('distance', Float32, callback)
    

    move_back_and_forth(group)

    rospy.spin()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        arm_control()
    except rospy.ROSInterruptException:
        pass
