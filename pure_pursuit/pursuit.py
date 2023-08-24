import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import os 
import sys
import numpy as np

from .ppp import PurePursuitPlanner


class Racecar(Node):

    def __init__(self):
        namespace = 'pure_pursuit'

        super().__init__(namespace)

        basic_qos = QoSProfile(depth=10)

        ### ROS2 Initalization Parameters
        drive_topic = self.declare_parameter('drive_topic', '/drive').value
        laser_topic = self.declare_parameter('laser_topic', '/scan').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        hz = self.declare_parameter('hz', 100).value
        
        ### Vehicle Model Parameters
        wheelbase = self.declare_parameter('wheelbase', 0.3302).value

        ### Driving Algorithm Parameters
        lookahead = self.declare_parameter('lookahead_distance', 0.82461887897713965).value
        sprint_speed = self.declare_parameter('sprint_speed', 4.0).value
        v_gain = self.declare_parameter('velocity_gain', 1.375).value 
        max_reacquire = self.declare_parameter('max_reacquire', 20.0).value
        waypoint_path = self.declare_parameter('waypoint_path', "").value

        assert isinstance(drive_topic, str), "drive_topic parameter must be a str"
        assert isinstance(laser_topic, str), "laser_topic parameter must be a str"
        assert isinstance(odom_topic, str), "odom_topic parameter must be a str"
        assert isinstance(hz, int), "hz parameter must be a int"
        assert isinstance(wheelbase, float), "wheelbase parameter must be a float"
        assert isinstance(lookahead, float), "lookahead distance must be a float"
        assert isinstance(sprint_speed, float), "sprint speed must be a float"
        assert isinstance(v_gain, float), "velocity gain must be a float"
        assert isinstance(max_reacquire, float), "max reacquire must be a float"
        assert isinstance(waypoint_path, str), "waypoint path must be a str"

        # Check waypoint file is exist
        try:
            for path in sys.path:
                if namespace in path:
                    wpt = os.path.join(path.split("lib")[0], "share", namespace, waypoint_path)
                    if os.path.isfile(wpt):
                        waypoint_path = wpt
        except (FileNotFoundError, Exception):
            self.get_logger().error("Cannot load waypoint file.")

        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, basic_qos)
        self.sub = {
            'odom': self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data)
        }

        self.wheelbase = wheelbase
        self.Hz = hz
        self.timer = self.create_timer((1/self.Hz), self.publish_callback)

        self.planner = PurePursuitPlanner(
            ld=lookahead,
            wb=wheelbase,
            ss=sprint_speed,
            vg=v_gain,
            mr=max_reacquire,
            wp=waypoint_path
        )

        self.odom_data = None



    def publish_callback(self, _finally=None):
        """
        Publish to /drive topic.

        :return: None
        """

        if not self.odom_data:
            self.get_logger().warn("No Odometry data. Skip this callback")
            return 
        
        msg = AckermannDriveStamped()
        obs = self.odom_data

        if hasattr(self.planner, 'plan'):
            speed, steer = self.planner.plan(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        elif hasattr(self.planner, 'driving'):
            speed, steer = self.planner.driving(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        else:
            self.get_logger().error("Planner doesn't have `plan` or `driving` function.")
            speed, steer = 0.0, 0.0

        if _finally:
            self.get_logger().info("SIGINT. stopping the vehicle")
            speed, steer = 0.0, 0.0
        

        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.get_logger().info(f"speed: {speed}, steer: {steer}")

        self.pub.publish(msg)


    def odom_callback(self, msg: Odometry):
        """
        Update self Odomerty data.

        :param msg: nav_msgs.msg/Odometry
        :return: class variable update
        """
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.odom_data = {
            'pose_x': msg.pose.pose.position.x,
            'pose_y': msg.pose.pose.position.y,
            'pose_theta': np.arctan2(siny_cosp, cosy_cosp),
            'linear_vels_x': msg.twist.twist.linear.x,
            'linear_vels_y': msg.twist.twist.linear.y,
            'ang_vels_z': msg.twist.twist.angular.z
        }


def main(args=None):
    rclpy.init(args=args)
    racecar = Racecar()
    try:
        rclpy.spin(racecar)
    except KeyboardInterrupt:
        racecar.publish_callback(_finally=True)
    finally:
        racecar.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()