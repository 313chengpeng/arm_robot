import serial
import time
import struct

# 设置串口参数
# 注意在Linux下，默认普通用户没有串口读写权限，每次开机后对串口读写都要附加权限
# sudo chmod 666 /dev/ttyUSB0

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

ser = serial.Serial(port, baudrate, timeout=timeout)
try:
    # 打开串口
    # ser = serial.Serial(port, baudrate, timeout=timeout)

    if ser.is_open:
        print(f"串口 {port} 已打开，波特率为 {baudrate}")

    while True:
        # 发送指令
        ser.write(hex_command)
        print(f"已发送指令: {hex_command.hex()}")

        start_time = time.time()
        received_data = False

        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if len(data) >= 5:  # 至少要包含设备地址、功能码、返回字节数和CRC
                    # 解析返回字节数
                    byte_count = data[2]
                    if len(data) >= 3 + byte_count + 2:  # 确保包含所有数据和CRC
                        distance_data = data[3:3 + byte_count]
                        crc_received = data[3 + byte_count:3 + byte_count + 2]
                        crc_calculated = calculate_crc(data[:-2])

                        if crc_received == crc_calculated:
                            # 解析16位数据区
                            if byte_count >= 2:
                                distance = struct.unpack('>H', distance_data[:2])[0]
                                print(f"接收到的数据: {data.hex()}")
                                print(f"距离: {distance} mm")
                                received_data = True
                        else:
                            print("CRC校验失败")
                if received_data:
                    break
            time.sleep(0.1)  # 每秒检查一次
        # while time.time() - start_time < 5:
        #     if ser.in_waiting > 0:
        #         data = ser.read(ser.in_waiting)
        #         if len(data) >= 5:  # 至少要包含设备地址、功能码、返回字节数和CRC
        #             # 解析返回字节数
        #             byte_count = data[2]
        #             if len(data) >= 3 + byte_count + 2:  # 确保包含所有数据和CRC
        #                 distance_data = data[3:3 + byte_count]
        #                 crc_received = data[3 + byte_count:3 + byte_count + 2]
        #                 crc_calculated = calculate_crc(data[:-2])

        #                 if crc_received == crc_calculated:
        #                     # 解析16位数据区
        #                     if byte_count >= 2:
        #                         distance = struct.unpack('>H', distance_data[:2])[0]
        #                         print(f"接收到的数据: {data.hex()}")
        #                         print(f"距离: {distance} mm")
        #                         received_data = True
        #                         break
        #                 else:
        #                     print("CRC校验失败")
        #         else:
        #             # print("接收到的数据不足，继续读取...")
        #             pass
        #     else:
        #         # print("等待数据...")
        #         pass
        #     time.sleep(0.1)  # 更频繁地检查

        if not received_data:
            print("5秒内未收到数据，停止程序")
            break

except serial.SerialException as e:
    print(f"无法打开串口 {port}: {e}")

finally:
    # 关闭串口
    if ser.is_open:
        ser.close()
        print(f"串口 {port} 已关闭")