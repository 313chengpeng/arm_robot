from rm_msgs.msg import Gripper_Pick, Gripper_Set
import rospy

# from std_msgs.msg import Bool

def main():
    rospy.init_node('catch_my_breath', anonymous=True)
    catch_pub = rospy.Publisher('/rm_driver/Gripper_Cmd', Gripper_Pick, queue_size=10)
    catch_position_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
    rospy.loginfo("等待机器人准备就绪...")
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        gripper_cmd = Gripper_Pick()
        gripper_cmd.force = 100  # 1~1000，代表手爪夹持力，最大1.5kg
        gripper_cmd.speed = 500  # 1~1000，代表手爪开合速度，无量纲
        catch_pub.publish(gripper_cmd)
        
        grip_msg = Gripper_Set()
        grip_msg.position = 300  # 设置手爪关闭位置
        catch_position_pub.publish(grip_msg)
        rospy.loginfo("抓取命令发布完成，等待5秒后释放")
        rospy.sleep(5)
        
        grip_msg.position = 1000  # 设置手爪打开位置
        catch_position_pub.publish(grip_msg)
        rospy.loginfo("释放命令发布完成，等待5秒后再抓取")
        rospy.sleep(5)
    
    rospy.spin()

if __name__ == "__main__":
    main()
