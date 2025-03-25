import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import serial
import numpy as np
import math

class IMUReader(Node):

    def __init__(self):
        super().__init__('imu_reader')
        self.time_step = 0.05
        self.publisher_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.publisher_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(self.time_step, self.serial_callback)
        self.get_logger().info('IMU Reader Node has been started.')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        self.acc = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.mag = [0, 0, 0]
        self.position = np.array([0, 0, 0])
        self.orientation = 0
        self.velocity = np.array([0, 0, 0])

    def update(self, acc, gyro, mag, dt): 
        
        # compute orientation as a quaternion - do not use magnetometer
        # orientation = self.updateOrientation(acc, gyro, mag, dt)
        # compute orientation as a euler angles
        orientation = self.updateOrientation(acc, gyro, mag, dt)
        # compute position
        position = self.updatePosition(acc, gyro, mag, dt)
        return position, orientation

    def computePose(self):
        donothing = 0

    def printImuData(self):
        self.get_logger().info(f'ACC: {self.acc}')
        self.get_logger().info(f'GYRO: {self.gyro}')
        self.get_logger().info(f'MAG: {self.mag}')

    def serial_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').rstrip()
            if line:
                data = line.split(',')
                if len(data) == 4 and data[0] in ['acc', 'gyro', 'mag']:
                    if data[0] == 'acc':
                        self.acc = [float(data[1]), float(data[2]), float(data[3])]
                    elif data[0] == 'gyro':
                        self.gyro = [float(data[1]), float(data[2]), float(data[3])]
                    elif data[0] == 'mag':
                        self.mag = [float(data[1]), float(data[2]), float(data[3])]
                    
                    self.printImuData()
                    # Publish data
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu'
                    msg.linear_acceleration.x = self.acc[0]
                    msg.linear_acceleration.y = self.acc[1]
                    msg.linear_acceleration.z = self.acc[2]
                    msg.angular_velocity.x = self.gyro[0]
                    msg.angular_velocity.y = self.gyro[1]
                    msg.angular_velocity.z = self.gyro[2]

                    self.publisher_imu.publish(msg)

                    msg_mag = MagneticField()
                    msg_mag.header.stamp = self.get_clock().now().to_msg()
                    msg_mag.header.frame_id = 'imu'
                    msg_mag.magnetic_field.x = self.mag[0]
                    msg_mag.magnetic_field.y = self.mag[1]
                    msg_mag.magnetic_field.z = self.mag[2]

                    self.publisher_mag.publish(msg_mag)

                    #self.get_logger().info(f'Publishing: {line}')                    

                else:
                    self.get_logger().warn(f'Incomplete or malformed data: {line}')


            else:
                self.get_logger().warn('Empty line received')
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()