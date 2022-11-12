#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import math

from nav_msgs.msg import Odometry, Path

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

x_arr = []
y_arr = []

path_x = []
path_y = []

first_itr = True

def odom_callback(odometry: Odometry):
    global x_arr
    global y_arr
    x_arr.append(odometry.pose.pose.position.x)
    y_arr.append(odometry.pose.pose.position.y)

def path_callback(path: Path):
    global path_x, path_y
    global first_itr
    poses = path.poses
    for poseStamped in poses:
        path_x.append(poseStamped.pose.position.x) 
        path_y.append(poseStamped.pose.position.y)

if __name__ == '__main__':
    
    rospy.init_node('live_plotter')

    odom_subscriber = rospy.Subscriber('/odom', Odometry, callback=odom_callback)
    path_subscriber = rospy.Subscriber('/path', Path, callback=path_callback)
    x = []
    y = []
    for theta in range(0, 628, 50):
        x.append(theta/100)
        y.append(math.sin(theta/100))

    def animate(i):
        ax1.clear()
        plt.xlim([-1, 7])
        plt.ylim([-2, 2])

        ax1.scatter(x, y, marker='.', c='g', label='reference')
        ax1.plot(x_arr, y_arr, c='black', label='trajectory')
        ax1.legend()        

    ani = animation.FuncAnimation(fig, animate, interval=500)
    plt.show()

