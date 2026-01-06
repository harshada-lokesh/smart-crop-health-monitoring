#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

# Scenario data
plants = [
    (0.3, 0.3), (0.6, 0.6), (0.8, 0.85), (0.45, 0.35), (0.2, 0.2)
]
health_status = [0.2, 0.6, 0.9, 0.3, 0.75]  # 0 = poor, 1 = healthy

obstacles = [
    (0.6, 0.6), (0.2, 0.8), (0.5, 0.75), (0.35, 0.25)
]

robot_pose = {'x': 0.0, 'y': 0.0}
path_points = []

def odom_callback(msg):
    robot_pose['x'] = msg.pose.pose.position.x
    robot_pose['y'] = msg.pose.pose.position.y

def make_marker(id, x, y, color, shape=Marker.SPHERE, scale=0.05, ns="plants"):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = shape
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = 0.02
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    return marker

def get_health_color(health):
    if health < 0.3:
        return (1.0, 0.0, 0.0)  # Red - poor
    elif health < 0.7:
        return (1.0, 1.0, 0.0)  # Yellow - moderate
    else:
        return (0.0, 1.0, 0.0)  # Green - healthy

def publish_markers():
    rospy.init_node("plant_visualizer")
    pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        id = 0

        # Depot (e.g., starting point)
        depot = make_marker(id, 0, 0, (1.0, 1.0, 0.0), shape=Marker.CUBE, scale=0.08, ns="depot")
        pub.publish(depot)
        id += 1

        # Plants
        for i, (x, y) in enumerate(plants):
            color = get_health_color(health_status[i])
            plant_marker = make_marker(id, x, y, color)
            pub.publish(plant_marker)
            id += 1

        # Obstacles
        for x, y in obstacles:
            obs = make_marker(id, x, y, (0.5, 0.5, 0.5), shape=Marker.CYLINDER, scale=0.1, ns="obstacles")
            pub.publish(obs)
            id += 1

        # Robot marker
        robot_marker = make_marker(id, robot_pose['x'], robot_pose['y'], (0.0, 0.0, 1.0), scale=0.03, ns="robot")
        pub.publish(robot_marker)
        id += 1

        # Path trail
        pt = Point()
        pt.x = robot_pose['x']
        pt.y = robot_pose['y']
        pt.z = 0
        path_points.append(pt)

        path_marker = Marker()
        path_marker.header.frame_id = "odom"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "path"
        path_marker.id = id
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.015
        path_marker.color.r = 0.0
        path_marker.color.g = 0.4
        path_marker.color.b = 1.0
        path_marker.color.a = 1.0
        path_marker.points = path_points

        pub.publish(path_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass
