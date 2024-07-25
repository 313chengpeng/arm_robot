import rospy
from std_msgs.msg import Float32
import serial
import time
import struct
# 设置串口参数
# 注意在Linux下，默认普通用户没有串口读写权限，每次开机后对串口读写都要附加权限
# sudo chmod 666 /dev/ttyUSB0
# 设置串口参数
port = '/dev/ttyUSB0'  # 根据实际情况修改
baudrate = 9600  # 波特率，根据实际情况修改
timeout = 1  # 读取超时时间，单位：秒

# 要发送的16进制指令
hex_command = bytes.fromhex('01 03 01 00 00 01 85 F6')

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

def read_distance():
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        if ser.is_open:
            rospy.loginfo(f"串口 {port} 已打开，波特率为 {baudrate}")

        ser.write(hex_command)
        rospy.loginfo(f"已发送指令: {hex_command.hex()}")

        start_time = time.time()
        received_data = False

        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if len(data) >= 5:  # 至少要包含设备地址、功能码、返回字节数和CRC
                    byte_count = data[2]
                    if len(data) >= 3 + byte_count + 2:  # 确保包含所有数据和CRC
                        distance_data = data[3:3 + byte_count]
                        crc_received = data[3 + byte_count:3 + byte_count + 2]
                        crc_calculated = calculate_crc(data[:-2])

                        if crc_received == crc_calculated:
                            if byte_count >= 2:
                                distance = struct.unpack('>H', distance_data[:2])[0]
                                rospy.loginfo(f"接收到的数据: {data.hex()}")
                                rospy.loginfo(f"距离: {distance} mm")
                                received_data = True
                                return distance
                        else:
                            rospy.logwarn("CRC校验失败")
            time.sleep(0.6)  # 每0.6秒检查一次
        

        if not received_data:
            rospy.logwarn("5秒内未收到数据")

    except serial.SerialException as e:
        rospy.logerr(f"无法打开串口 {port}: {e}")

    finally:
        if ser.is_open:
            ser.close()
            rospy.loginfo(f"串口 {port} 已关闭")

def distance_publisher():
    rospy.init_node('distance_publisher', anonymous=True)
    pub = rospy.Publisher('distance', Float32, queue_size=1) # queue_size表示缓存队列的大小
    # rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        distance = read_distance()
        if distance is not None:
            distance_mm = distance/1000 # 将mm转为m
            pub.publish(distance_mm)  # 发布距离
            rospy.loginfo(f"发布距离: {distance} mm")
        # rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass
