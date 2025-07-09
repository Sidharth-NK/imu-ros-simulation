import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from dfrobot_imu_ahrs_driver.DFRobot_BMX160 import BMX160
from dfrobot_imu_ahrs_driver.madgwick_py.madgwickahrs import MadgwickAHRS

class BMX160Publisher(Node):
    def __init__(self):
        super().__init__('bmx160_imu_publisher')
        self.imu = BMX160(bus=1)

        if not self.imu.begin():
            self.get_logger().error("Failed to initialize BMX160 IMU sensor")
            exit(1)

        time.sleep(1)  # stabilize sensor

        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.01, self.publish_imu_data)
        self.filter = MadgwickAHRS(sampleperiod=0.01)
        self.get_logger().info("BMX160 IMU publisher node started")

    def publish_imu_data(self):
        data = self.imu.get_all_data()
        mag = {'x': data[0], 'y': data[1], 'z': data[2]}
        gyro = {'x': data[3], 'y': data[4], 'z': data[5]}
        accel = {'x': data[6], 'y': data[7], 'z': data[8]}

        gx, gy, gz = np.radians([gyro['x'], gyro['y'], gyro['z']])

        self.filter.update(
            [gx, gy, gz],
            [accel['x'], accel['y'], accel['z']],
            [mag['x'], mag['y'], mag['z']]
        )
        q = self.filter.quaternion.q  # [w, x, y, z]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.w = float(q[0])
        msg.orientation.x = float(q[1])
        msg.orientation.y = float(q[2])
        msg.orientation.z = float(q[3])

        msg.angular_velocity = Vector3(x=float(gx), y=float(gy), z=float(gz))
        msg.linear_acceleration = Vector3(
            x=float(accel['x']), y=float(accel['y']), z=float(accel['z'])
        )

        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        self.publisher_.publish(msg)
        self.get_logger().debug("Published IMU data")

def main(args=None):
    rclpy.init(args=args)
    node = BMX160Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
