#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from laser_values.msg import Lidar_data
import numpy as np
new=[0,0,0,0]
p_new=[10,10,10,10]
j=1
k=0
dist=0



out=Lidar_data()

'''def left():  
	out.linear.x=0
	out.angular.z=10
	print("left")

def right():
	out.linear.x=0
	out.angular.z=-10
	print("right")


def forward():
	out.linear.x=5
	out.angular.z=0
	print("forward")

def forward_slow():
	out.linear.x=5
	out.angular.z=0
	print("forward slow")

def backward():
	out.linear.x=-5
	out.angular.z=0
	print("backward")

def stop():
	out.linear.x=0
	out.angular.z=0
	print("stop")

def callback2(msg,pub):

	global dist
	dist=msg.data

	if(dist==65536):
		dist=-400

	dist=dist/1000
'''	



def callback1(msg,pub):
	global dist
	global new
	global p_new
	global j
	global k

	arr = msg.ranges
	arr = np.where(np.isnan(arr),10,arr)
	
	for k in range(0,4):
		for j in range(1,160):
			if(arr[(160*k)+j]==10):
				arr[(160*k)+j]=p_new[k]

	
	
##...........................obstacle...................................##
	for k in range(0,4):
		min=arr[(160*k+j)]
		max=arr[(160*k+j)]
		for j in range(1,160):
			if(min>arr[(160*k)+j]):
				min=arr[(160*k)+j]
			if(max<arr[(160*k)+j]):
				max=arr[(160*k)+j]
		new[k]=min
		p_new[k]=max
	
	print (new)
	out.zone_0=new[0]
	out.zone_1=new[1]
	out.zone_2=new[2]
	out.zone_3=new[3]

	pub.publish(out)





	
	if(new[1]>1.5 and new[2]>1.5 ):
		forward()

	elif(new[0]<1 or  new[1]<1 or new[2]<1 or new[3]<1 ):
		backward()
		
	elif(new[2]<1.5 and new[3]<1.5):
		right()

	elif(new[0]<1.5 and new[1]<1.5 ):
		left()

	elif(new[0]<1.5):
		left()

	elif(new[1]<1.5):
		left()


	elif(new[2]<1.5):
		right()

	elif(new[3]<1.5):
		right()


rospy.init_node('scan_values')
pub = rospy.Publisher('/kinect_topic',Lidar_data,queue_size=5)
sub1=rospy.Subscriber('/scan',LaserScan,callback1,pub)
#sub2=rospy.Subscriber('/lidar',Float64,callback2,pub)
rospy.spin()




