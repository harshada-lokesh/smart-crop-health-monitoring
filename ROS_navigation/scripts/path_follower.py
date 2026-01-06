#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

waypoints = [
    [0.0, 0.0], [0.15, 0.1], [0.3, 0.25], [0.45, 0.35], [0.55, 0.45],
    [0.65, 0.55], [0.7, 0.65], [0.75, 0.75], [0.8, 0.85], [0.85, 0.9],
    [0.9, 0.95], [0.95, 1.0], [0.9, 0.9], [0.8, 0.8], [0.7, 0.7],
    [0.6, 0.6], [0.5, 0.5], [0.4, 0.4], [0.3, 0.3], [0.2, 0.2],
    [0.1, 0.1], [0.0, 0.0]
]

# Plant target positions (adjusted)
plant_targets = [
    [0.3, 0.3],
    [0.6, 0.6],
    [0.8, 0.85],
    [0.45, 0.35],
    [0.2, 0.2]
]

class PathFollower:
    def __init__(self):
        rospy.init_node('rosbot_path_follower', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.pose = {'x': 0, 'y': 0, 'theta': 0}
        self.rate = rospy.Rate(10)
        self.tolerance = 0.05
        self.reached_plants = [False] * len(plant_targets)

    def odom_callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.pose['theta'] = yaw

    def distance(self, x, y):
        return math.hypot(x - self.pose['x'], y - self.pose['y'])

    def angle_to_target(self, x, y):
        return math.atan2(y - self.pose['y'], x - self.pose['x'])

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def move_to_waypoint(self, x, y):
        vel = Twist()
        while not rospy.is_shutdown() and self.distance(x, y) > self.tolerance:
            angle = self.angle_to_target(x, y)
            angle_diff = self.normalize_angle(angle - self.pose['theta'])

            if abs(angle_diff) > 0.1:
                vel.linear.x = 0.0
                vel.angular.z = 0.5 * angle_diff
            else:
                vel.linear.x = min(0.2, self.distance(x, y))
                vel.angular.z = 0.0

            self.cmd_pub.publish(vel)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # stop

    def follow_path(self):
        rospy.sleep(2.0)
        rospy.loginfo("🌱 Starting plant path with %d points.", len(waypoints))

        for i, (x, y) in enumerate(waypoints):
            self.move_to_waypoint(x, y)

            # Check if any plant is reached
            for j, (px, py) in enumerate(plant_targets):
                if not self.reached_plants[j] and self.distance(px, py) < 0.05:
                    rospy.loginfo("🌿 Plant %d reached at (%.2f, %.2f)", j + 1, px, py)
                    self.reached_plants[j] = True

        rospy.loginfo("✅ All waypoints complete.")

if __name__ == '__main__':
    try:
        PathFollower().follow_path()
    except rospy.ROSInterruptException:
        pass
