#!/usr/bin/env python3	

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import sin, cos
import cubic_spline_planner

def shutdown_callback():
    rospy.loginfo("Shutting down path_publisher node.")

# Custom Subscriber Listener
# class MySubscribeListener(rospy.SubscribeListener):
#     def __init__(self) -> None:
#         super().__init__()

#     def peer_subscribe(self, topic_name, topic_publish, peer_publish):
#         return super().peer_subscribe(topic_name, topic_publish, peer_publish)

#     def peer_unsubscribe(self, topic_name, num_peers):
#         rospy.loginfo("One node has unsubscribed from topic {}. Total nodes remaining {}".format(topic_name, num_peers))
#         rospy.signal_shutdown()


rospy.init_node('path_publisher')
rospy.on_shutdown(shutdown_callback)

path_publisher = rospy.Publisher('/path', Path ,queue_size=1)

rate = rospy.Rate(1)

path = Path()

points = []
path_x = []
path_y = []

x = []
y = []

# for theta in range(625):
#     x.append(theta/100)
#     y.append(sin(theta))

# print("Length of path {}".format(len(x)))

# x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
# y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]

# path_x, path_y, path_yaw, ck, s = cubic_spline_planner.calc_spline_course(x, y, ds=0.1)

# print("Length of spline path {}".format(len(path_x)))

for theta in range(628):
    pose = PoseStamped()
    pose.pose.position.x = theta/100
    pose.pose.position.y = sin(theta)
    path.poses.append(pose)

print("Path of length {} points calculated".format(len(path.poses)))

if __name__ == '__main__':
    # Code goes here
    
    while not rospy.is_shutdown():
        path_publisher.publish(path)
        rate.sleep()
        