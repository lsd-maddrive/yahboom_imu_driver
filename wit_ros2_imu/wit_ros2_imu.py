import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

key = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data
    key += 1

    if buff[0] != 0x55:
        key = 0
        return False

    if key < 11:
        return False
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51 and check_sum(data_buff[0:10], data_buff[10]):
            acceleration[:] = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)]
        elif buff[1] == 0x52 and check_sum(data_buff[0:10], data_buff[10]):
            angularVelocity[:] = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
        elif buff[1] == 0x53 and check_sum(data_buff[0:10], data_buff[10]):
            angle_degree[:] = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)]
            angle_flag = True
        elif buff[1] == 0x54 and check_sum(data_buff[0:10], data_buff[10]):
            magnetometer[:] = hex_to_short(data_buff[2:10])

        buff.clear()
        key = 0
        return angle_flag


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__(node_name='imu_driver_node')

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port, baud))
        self.driver_thread.start()

    def driver_loop(self, port, baud):
        try:
            wt_imu = serial.Serial(port=port, baudrate=baud, timeout=0.5)
            self.get_logger().info(f"Serial port {port} opened at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Serial port open failed: {e}")
            return

        while True:
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                self.get_logger().error(f"Serial exception: {e}")
                return

            if buff_count > 0:
                buff_data = wt_imu.read(buff_count)
                for i in range(buff_count):
                    tag = handle_serial_data(buff_data[i])
                    if tag:
                        self.imu_data()

    def imu_data(self):
        accel_x, accel_y, accel_z = acceleration
        gyro_x, gyro_y, gyro_z = angularVelocity

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z

        angle_radian = [math.radians(deg) for deg in angle_degree]
        qua = get_quaternion_from_euler(*angle_radian)
        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        self.imu_pub.publish(self.imu_msg)
        self.publish_tf()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
            
if __name__ == "__main__":
    main()
