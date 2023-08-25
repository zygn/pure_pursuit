#!/usr/bin/python3

import rospy 
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from ppp import PurePursuitPlanner

class Racecar:

    def __init__(self) -> None:
        
        self.drive_topic = rospy.get_param('~drive_topic', '/drive')
        self.laser_topic = rospy.get_param('~scan_topic', '/scan')
        self.odom_topic = rospy.get_param('~odom_topic', '/vesc/odom')

        self.wheelbase = rospy.get_param('~wheelbase', 0.3302)
        self.lookahead = rospy.get_param('~lookahead_distance', 0.8)
        self.sprint_speed = rospy.get_param('~sprint_speed', 4.0)
        self.v_gain = rospy.get_param('~velocity_gain', 1.375)
        self.max_reacquire = rospy.get_param('~max_reacquire', 20.0)
        self.waypoint_path = rospy.get_param('~waypoint_path', "")

        self.pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        self.planner = PurePursuitPlanner(
            ld=self.lookahead,
            wb=self.wheelbase,
            ss=self.sprint_speed,
            vg=self.v_gain,
            mr=self.max_reacquire,
            wp=self.waypoint_path
        )

        self.odom_data = {}

    def publish_callback(self):

        if not self.odom_data:
            rospy.logwarn("No Odometry data. Skip this callback")
            return 

        msg = AckermannDriveStamped()
        obs = self.odom_data

        if hasattr(self.planner, 'plan'):
            speed, steer = self.planner.plan(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        elif hasattr(self.planner, 'driving'):
            speed, steer = self.planner.driving(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        else:
            rospy.logerr("Planner doesn't have `plan` or `driving` function.")
            speed, steer = 0.0, 0.0
        
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        rospy.loginfo(f"speed: {speed}, steer: {steer}")

        self.pub.publish(msg)


    
    def odom_callback(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)

        self.odom_data['pose_x'] = msg.pose.pose.position.x
        self.odom_data['pose_y'] = msg.pose.pose.position.y
        self.odom_data['pose_theta'] = current_position_theta
        self.odom_data['linear_vels_x'] = msg.twist.twist.linear.x


if __name__ == "__main__":
    rospy.init_node('pure_pursuit')
    rate = rospy.Rate(100)
    app = Racecar()
    while not rospy.is_shutdown():
        app.publish_callback()
        rate.sleep()
