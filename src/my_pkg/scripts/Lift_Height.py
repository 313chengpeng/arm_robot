# !/usr/bin/env python
# -*- coding: utf-8 -*-

# Lift_Height
# 控制机械臂高度闭环控制

import rospy
from rm_msgs.msg import MoveL, MoveJ_P, Plan_State,Gripper_Pick,Gripper_Set
from rm_msgs.msg import Lift_Height,Plan_State, LiftState, Stop
from rm_msgs.msg import GetArmState_Command, Arm_Current_State, ArmState
from std_msgs.msg import Bool,Float32,Byte

# Lift_Height.msg
# #升降机构运行到指定高度
# uint16 height #目标高度，单位 mm，范围：0~2600
# uint16 speed #速度百分比，1~100

# LiftState.msg
# int16 height #当前高度
# int16 current #当前电流
# uint16 err_flag #驱动错误代码
# byte mode #当前升降状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置

sensor_target_height = 250/1000 # 设定高度270mm
safe = 10/1000  # 容差为20 mm
dangerous = 200/1000 # 偏差大于60mm有点危险，控制断电用
bias = 20/1000      # 这个偏差可以先通过一次测量进行校准，后面的使用可以仅依据传感器测量
err = 0     # 也可以通过偏差的变化量作为调整的依据
# arm_target_height = sensor_target_height + bias # 例如机械臂读取自身的高度为30厘米，传感器测量到的为28厘米
arm_current_height = 290/1000 # 假设当前的机械臂高度为270，后续在读取后更新
set_height = (sensor_target_height+bias)/1000
move_completed = False
catch_flag = 0

move_state = 0
# 用于判断机械臂动作阶段
# 1 向point2点运动
# 2 向point1点运动
# 3 Lift_Height高度上的调整
# 4 
pre_state = 0
# 存储前一阶段的运动状态，用于高度调整后，重新设定目标点位


def adjust_height(data):
    # 传入的数据为 err 单位m
    global arm_current_height,set_height,move_state,pre_state,pose_data
    # rospy.loginfo('即将停止运动，调整高度')
    # stop_cmd = Stop()
    # stop_cmd.state = True
    # stop_pub.publish(stop_cmd)
    # rospy.loginfo('停止运动指令已发布')
    query_arm_state()
    rospy.sleep(0.1) 
    rospy.loginfo('正在查询机械臂当前状态：')
    rospy.loginfo(f'查询结果：arm_current_height={arm_current_height}')
    set_height = arm_current_height - data
    rospy.loginfo(f'set_height(adjust)= {set_height}')
    # set_height_cmd = Lift_Height()
    # set_height_cmd.height = int(set_height*1000)
    # set_height_cmd.speed = 30
    moveL_target_pose_2 = MoveL()
    moveL_target_pose_2.Pose = pose_data
    moveL_target_pose_2.Pose.position.z = set_height
    moveL_target_pose_2.speed = 0.1
    moveL_pub.publish(moveL_target_pose_2)
    rospy.loginfo(f'更新运动指令,向point2移动保持高度为{set_height*1000}mm')
    rospy.loginfo(f"moveL_target_pose 更新目标：{moveL_target_pose_2}")
    
    
    
    # rospy.loginfo(f'设置机械臂高度为{set_height_cmd.height}mm')
    # stable_height_pub.publish(set_height_cmd)
    rospy.loginfo('等待高度调整稳定')
    rospy.sleep(0.1)

def sensor_callback(data):
    # 传入的距离数据的单位为 m
    global err,move_state,pre_state
    sensor_current_height = data.data
    # rospy.loginfo(f"当前传感器获取高度：{sensor_current_height*1000}mm")
    err = sensor_current_height - sensor_target_height  # 当前的高度减去目标高度，如果遇到障碍物，那么当前高度会减小，那么err就是负数，而对应的机械臂应该向上调整，即在原始的高度上减去当前偏差
    # rospy.loginfo(f'与目标设定的偏差为{err*1000}mm')
    # if abs(err) > dangerous:
    #     # 如果突然间误差过大，机械臂断电保护
    #     power_pub =rospy.Publisher('/rm_driver/SetArmPower',Byte,queue_size=10)
    #     power_flag = 0
    #     rospy.logerr('误差过大，断电保护')
    #     # power_pub.publish(power_flag)
    #     adjust_height(err)
    if abs(err) > safe:
    # elif abs(err) > safe:
        if move_state == 1 or move_state == 2:
            rospy.loginfo(f"当前传感器获取高度：{sensor_current_height*1000}mm")
            rospy.loginfo(f'与目标设定的偏差为{err*1000}mm')
            rospy.loginfo("当前高度小于安全距离，即将进行调整")
            rospy.loginfo('即将停止运动，调整高度')
            stop_cmd = Stop()
            stop_cmd.state = True
            stop_pub.publish(stop_cmd)
            rospy.loginfo('停止运动指令已发布')
            pre_state =  move_state   
            move_state = 3  
            adjust_height(err)

def plan_state_callback(data):
    global move_completed,catch_flag,set_height,move_state  
    if move_state == 1 or move_state == 2:
        if data.state:
            rospy.loginfo('运动指令执行完毕')

            if  catch_flag == 1:
                rospy.loginfo("发布抓取命令")
                gripper_cmd = Gripper_Pick()
                gripper_cmd.force = 500  # 1~1000，代表手爪夹持力，最大1.5kg
                gripper_cmd.speed = 800  # 1~1000，代表手爪开合速度，无量纲
                catch_pub.publish(gripper_cmd)
                
                grip_msg = Gripper_Set()
                grip_msg.position = 1  # 设置手爪关闭位置
                catch_position_pub.publish(grip_msg)
                rospy.loginfo("抓取命令发布完成，等待释放")
                rospy.sleep(3)
                catch_flag = 0 # 仅执行一次
            # elif catch_flag == 2:
            #     rospy.loginfo("发布释放命令")
            #     grip_msg = Gripper_Set()
            #     grip_msg.position = 1000  # 设置手爪打开位置
            #     catch_position_pub.publish(grip_msg)
            #     rospy.loginfo("释放命令发布完成，等待抓取")
            #     rospy.sleep(3)
        else:
            rospy.logerr('运动指令未成功执行')
    if  move_state == 3:
        if data.state:
            rospy.loginfo('运动指令执行完毕')
            move_state = pre_state
            if move_state ==1:
                moveL_target_pose_2 = MoveL()
                moveL_target_pose_2.Pose.position.x = -0.2845299
                moveL_target_pose_2.Pose.position.y = 0.31842098
                moveL_target_pose_2.Pose.position.z = set_height
                moveL_target_pose_2.Pose.orientation.x = 0.6501861
                moveL_target_pose_2.Pose.orientation.y = -0.759086
                moveL_target_pose_2.Pose.orientation.z = -0.031065
                moveL_target_pose_2.Pose.orientation.w = -0.008943
                moveL_target_pose_2.speed = 0.1
                move_state = 2
                moveL_pub.publish(moveL_target_pose_2)
                rospy.loginfo(f'更新运动指令,向point2移动保持高度为{set_height*1000}mm')
            elif move_state ==2:
                moveL_target_pose = MoveL()
                moveL_target_pose.Pose.position.x = 0.2545299
                moveL_target_pose.Pose.position.y = 0.31842098
                moveL_target_pose.Pose.position.z = set_height
                moveL_target_pose.Pose.orientation.x = 0.6501861
                moveL_target_pose.Pose.orientation.y = -0.759086
                moveL_target_pose.Pose.orientation.z = -0.031065
                moveL_target_pose.Pose.orientation.w = -0.008943
                moveL_target_pose.speed = 0.1
                move_state = 1
                moveL_pub.publish(moveL_target_pose)
                rospy.loginfo(f'更新运动指令,向point2移动保持高度为{set_height*1000}mm')
                
        else:
            rospy.logerr('高度指令未成功执行')
            rospy.sleep(0.1)

def arm_state_callback(data):
    # 读取当前机械臂的高度 单位m
    global arm_current_height, pose_data
    rospy.loginfo("已成功接收机械臂位置信息，正在解析...")
    print("Joint angles: ", data.joint)
    print("Pose: ", data.Pose)
    print("Arm error: ", data.arm_err)
    print("System error: ", data.sys_err)
    print("Degrees of freedom: ", data.dof)
    arm_current_height = data.Pose.position.z
    pose_data = data.Pose
    rospy.loginfo(f"机械臂状态解析完毕,arm_current_height={arm_current_height*1000}mm")


def query_arm_state():
    query_arm_state_cmd = GetArmState_Command()
    query_arm_state_cmd.command = ''
    query_arm_state_pub.publish(query_arm_state_cmd.command)
    rospy.Subscriber('/rm_driver/ArmCurrentState', ArmState, arm_state_callback)
    rospy.loginfo('机械臂状态查询信息发布完毕')

def my_main():
    global set_height,catch_flag,move_state
    try:
        rospy.loginfo("Waiting for the robot to be ready...")
        rospy.sleep(2.0)
        rospy.loginfo("即将运动至预定点位")

        moveJ_P_target_pose = MoveJ_P()
        moveJ_P_target_pose.Pose.position.x = -0.0245299
        moveJ_P_target_pose.Pose.position.y = 0.31842098
        moveJ_P_target_pose.Pose.position.z = 0.315527
        moveJ_P_target_pose.Pose.orientation.x = 0.6501861
        moveJ_P_target_pose.Pose.orientation.y = -0.759086
        moveJ_P_target_pose.Pose.orientation.z = -0.031065
        moveJ_P_target_pose.Pose.orientation.w = -0.008943
        moveJ_P_target_pose.speed = 0.3

        moveJ_P_pub.publish(moveJ_P_target_pose)
        rospy.sleep(5)
        rospy.loginfo("The first trajectory has been executed")
        rospy.loginfo("Prepare to execute Instruction MoveL")
        
        
        moveL_target_pose = MoveL()
        moveL_target_pose.Pose.position.x = -0.0245299
        moveL_target_pose.Pose.position.y = 0.31842098
        moveL_target_pose.Pose.position.z = 0.290527
        moveL_target_pose.Pose.orientation.x = 0.6501861
        moveL_target_pose.Pose.orientation.y = -0.759086
        moveL_target_pose.Pose.orientation.z = -0.031065
        moveL_target_pose.Pose.orientation.w = -0.008943
        moveL_target_pose.speed = 0.1
        moveL_pub.publish(moveL_target_pose)
        rospy.loginfo('等待机械臂稳定')
        rospy.sleep(5)
        grip_msg = Gripper_Set()
        grip_msg.position = 1000  # 设置手爪打开位置
        catch_position_pub.publish(grip_msg)
        rospy.loginfo("等待初始化夹爪")
        catch_flag = 1      # 设置夹爪的标志位，
        rospy.sleep(5)
    
        # moveL_target_pose_2 = MoveL()
        # moveL_target_pose_2 = moveL_target_pose
        # moveL_target_pose_2.Pose.position.x =  -0.1245299
        
        
        rospy.Subscriber('distance', Float32, sensor_callback)
        rospy.loginfo('回调启用')
        while not rospy.is_shutdown():
            rospy.loginfo('机械臂即将运动至point2')
            # moveL_target_pose_2 = MoveJ_P()
            moveL_target_pose_2 = MoveL()
            moveL_target_pose_2.Pose.position.x = -0.2845299
            moveL_target_pose_2.Pose.position.y = 0.31842098
            rospy.sleep(3) 
            moveL_target_pose_2.Pose.position.z = set_height
            moveL_target_pose_2.Pose.orientation.x = 0.6501861
            moveL_target_pose_2.Pose.orientation.y = -0.759086
            moveL_target_pose_2.Pose.orientation.z = -0.031065
            moveL_target_pose_2.Pose.orientation.w = -0.008943
            moveL_target_pose_2.speed = 0.1

            move_state = 1  
            # moveJ_P_pub.publish(moveL_target_pose_2)
            
            moveL_pub.publish(moveL_target_pose_2)
            
            rospy.loginfo('point2目标发布')
            rospy.sleep(25)
            
            rospy.loginfo('机械臂即将返回至point1')
            # moveL_target_pose = MoveJ_P()
            moveL_target_pose = MoveL()
            moveL_target_pose.Pose.position.x = 0.2545299
            moveL_target_pose.Pose.position.y = 0.31842098
            rospy.sleep(3) 
            moveL_target_pose.Pose.position.z = set_height
            moveL_target_pose.Pose.orientation.x = 0.6501861
            moveL_target_pose.Pose.orientation.y = -0.759086
            moveL_target_pose.Pose.orientation.z = -0.031065
            moveL_target_pose.Pose.orientation.w = -0.008943
            moveL_target_pose.speed = 0.3
            # catch_flag = 2

            # moveJ_P_pub.publish(moveL_target_pose)
            moveL_pub.publish(moveL_target_pose)
            rospy.loginfo('point1目标发布')
            rospy.sleep(25)    
        
    except rospy.ROSInterruptException:
        rospy.loginfo('问题')
        pass

if __name__ == '__main__':
    rospy.init_node('Stable_height', anonymous=True)
    stop_pub = rospy.Publisher('/rm_driver/Emergency_Stop', Stop, queue_size=1)
    stable_height_pub = rospy.Publisher('/rm_driver/Lift_SetHeight', Lift_Height, queue_size=1)
    query_arm_state_pub  = rospy.Publisher('/rm_driver/GetArmState_Cmd', GetArmState_Command, queue_size=2)
    moveJ_P_pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=1)
    moveL_pub = rospy.Publisher('/rm_driver/MoveL_Cmd', MoveL, queue_size=1)
    catch_pub = rospy.Publisher('/rm_driver/Gripper_Cmd', Gripper_Pick, queue_size=10)
    catch_position_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
    # Subscribe to the robot arm execution state topic
    plan_state_sub = rospy.Subscriber('/rm_driver/Plan_State', Plan_State, plan_state_callback)

    my_main()
