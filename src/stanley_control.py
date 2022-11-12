#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import math


rospy.init_node('stanley_control')

ctr = 1
brk = 0
L = 0.3
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

MAX_ANGULAR_VELOCITY = 0.5
MAX_LINEAR_VELOCITY = 0.2

x_arr = []
y_arr = []
counter = 0
counter_arr = []
c_error_arr = []
h_error_arr = []
delta_arr = []
omega_arr = []

def shutdown_callback():
	pub.publish(Twist())

def cb(data):
	# counter to run through the elements of the list
	global ctr, brk
	global x_arr, y_arr
	global counter_arr, counter
	global c_error_arr, h_error_arr, delta_arr, omega_arr
	vel = Twist()

	# subject to testing
	proportional_gain = 2.5

	# current x and y position and the angle of the axle
	curr_pos_x = data.pose.pose.position.x
	curr_pos_y = data.pose.pose.position.y
	angles = [data.pose.pose.orientation.x, data.pose.pose.orientation.y ,data.pose.pose.orientation.z, data.pose.pose.orientation.w]
	(roll, pitch, curr_angle_z) = euler_from_quaternion(angles)

	x_arr.append(curr_pos_x)
	y_arr.append(curr_pos_y)


	# simple point to point had INSANE CROSS-TRACK ERROR
	# trying to eliminate that using Stanley
	# angle_to_turn = difference_in_angle + arctan(k * lateral_error / (softening_const * vel))

	# for lateral error, we need shortest distance between line and trajectory.

	slope = (arr[ctr][1] - arr[ctr-1][1])/(arr[ctr][0] - arr[ctr-1][0] + 0.0000000001)

	# (y - y1) / (y2 - y1) = (x - x1) / (x2 - x1)

	# the C of y = mx + C
	intercept_line = -slope * arr[ctr-1][0] + arr[ctr-1][1]

	line1 = [1, -slope]
	line2 = [1, 1/slope]

	# the C of y = mx + C
	intercept_point = 1/slope * curr_pos_x + curr_pos_y

	A = np.array([line1, line2])
	B = np.array([intercept_line, intercept_point])

	# x and y that are at the shortest distnace
	(short_y, short_x) = np.linalg.solve(A,B)
	print(short_y)
	sign_of_error = 1

	if short_y > curr_pos_y:
		sign_of_error = 0

	cross_track_error = ((-1)**sign_of_error) * math.sqrt((curr_pos_x - short_x)**2 + (curr_pos_y - short_y)**2)

	c_error_arr.append(cross_track_error)
	
	error = 0.05

	if ( abs(curr_pos_x - arr[-1][0]) <= error and abs(curr_pos_y - arr[-1][1]) <= error):
		vel.linear.x = 0
		vel.angular.z = 0
		print("Goal Reached")
		pub.publish(vel)
		# plt.plot(x_arr, y_arr, c='black')
		# for i in range(len(arr)):
			# plt.scatter(x=arr[i][0],y=arr[i][1], c='y', marker=',')
		
		# plt.show()
		rospy.signal_shutdown("Goal Reached")

	elif ctr == 1 and brk == 0:
		delta = math.atan2(arr[ctr][1] - arr[ctr-1][1],arr[ctr][0] - arr[ctr-1][0] + 0.0000000001) - curr_angle_z
		if abs(delta)>error:
			vel.linear.x = 0.0
			delta = math.atan2(arr[ctr][1] - arr[ctr-1][1],arr[ctr][0] - arr[ctr-1][0] + 0.0000000001) - curr_angle_z
			vel.angular.z = delta
		else: 
			vel.linear.x = MAX_LINEAR_VELOCITY
			brk = 1
	
	else:
		vel.linear.x = MAX_LINEAR_VELOCITY

		# find the angle between the trajectory heading and vehicle heading

		phi = math.atan2(arr[ctr][1] - arr[ctr-1][1],arr[ctr][0] - arr[ctr-1][0] + 0.0000000001) - curr_angle_z

		h_error_arr.append(phi)

		# if(phi < 0):
		# 	cross_track_error = -1 * cross_track_error
		
		delta = phi + math.atan2((proportional_gain * cross_track_error) , 0.0000000001 + vel.linear.x)
		print("Delta: ", delta)
		delta_arr.append(delta)

		# if arr[ctr][1] < 0 and arr[ctr][0] < 0:
		#	delta = delta - math.pi

		if delta > MAX_ANGULAR_VELOCITY:
			delta = (MAX_ANGULAR_VELOCITY)

		elif delta < - MAX_ANGULAR_VELOCITY:
			delta = - (MAX_ANGULAR_VELOCITY)
		
		omega = vel.linear.x * math.sin(delta/L)
		#to be uncommented for cytron
		#omega = omega * MAX_ANGULAR_VELOCITY/ (math.pi/4)
		vel.angular.z = omega

		omega_arr.append(omega)
		counter_arr.append(counter)
		counter += 1

		if (abs(curr_pos_x - arr[ctr][0]) <= error and abs(curr_pos_y - arr[ctr][1]) <= error):
			if ctr != len(arr):
				ctr += 1
		# if abs(curr_pos_x) > abs(arr[ctr][0]) and abs(curr_pos_y) > abs(arr[ctr][1]):
		# 	if ctr != len(arr):
		# 		ctr += 1
	print(ctr)
	pub.publish(vel)	
	
arr = []
	
# arr =[[0,0],[1.5,-0.9],[2,-0.7],[2.8,-0.4],[3.5,0.7],[4.2,0.8],[4.9,1.0],[5.5, 0.7],[6.2,0.5],[7,-0.4]]


# for i in range(300):
# 	x = 2*i/100
# 	y = 0.25 * x**2
# 	arr.append([x, y])

divisor = 200	
# for i in range(divisor):
# 	# y^2 = 2x - x^2
# 	x = (i/divisor)
# 	y = math.sqrt(abs(2*x - x**2))
# 	arr.append([x, y])

radius = 4

a = 2
b = 3
origin = 2
# for theta in range(0, 314, 2):
# # # 	# Circle
# # # 	# x = radius * (1- math.cos(theta/100))
# # # 	# y = radius * math.sin(theta/100)
#  	# Ellipse
# 	x = origin - a * math.cos(theta/100)
# 	y = b * math.sin(theta/100)
# 	arr.append([x, y])


# (x-o) ^ 2 / a^2 + y^2 = 1
# for i in range(100):
# 	x = i/100
# 	y = math.sqrt(abs(1- ((x-o)**2/a**2)))
# 	arr.append([x, y])

# Sin wave
# change xlims
for theta in range(628):
	
	x = theta/100
	y = math.sin(x)
	arr.append([x, y])

# arr = [[0,0],[1,1],[2,2],[3,3],[4,4],[5,5]]
# arr = [[0,0],[1,2.72],[1.5,4.48],[2,7.39],[3,20.086]]
path_publisher = rospy.Publisher('/path', Path, queue_size=1)
path = Path()
for i in range(len(arr)):
	poseStamped = PoseStamped()
	poseStamped.pose.position.x = arr[i][0]
	poseStamped.pose.position.y = arr[i][1]
	path.poses.append(poseStamped)

rospy.Subscriber('/odom', Odometry, cb)
rospy.on_shutdown(shutdown_callback)

print(path)
for i in range(10):
	path_publisher.publish(path)

rospy.spin()