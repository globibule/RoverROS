#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

import tf
import math
import time

exec_time = 1/25;

ThrottleChannel = 2
SteeringChannel = 0

throttleTrimPWM = 1500	#trim PWM
throttleMinPWM  = 1100  #min PWM
throttleMaxPWM  = 1800	#max PWM
throttleMultPWM = 20 	#multiplication

steeringTrimPWM = 1500
steeringMinPWM  = 1100
steeringMaxPWM  = 1800
steeringMultPWM = 20

RCOverride = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]

rangeUltrasonicFront = 4
rangeUltrasonicBack  = 4

def setMode():
	print("Set mode")
	rospy.wait_for_service('/mavros/set_mode')
	try:
		modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		modeResponse = modeService(0, 'MANUAL')
		rospy.loginfo("\nMode Response: " + str(modeResponse))
		print("Mode set")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

def arm():
	print("Set arming")
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		armResponse = armService(True)
		rospy.loginfo(armResponse)
		print("Armed")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

def disarm():
	print("Set disarming")
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		armResponse = armService(False)
		rospy.loginfo(armResponse)
		print("Disarmed")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

def subscribeToData():
	print("Subscribing")
	#rospy.Subscriber("/mavros/imu/data", Imu, getIMU)
	#rospy.Subscriber("/mavros/rc/in", RCIn, getRC)
	rospy.Subscriber("/ultrasound1", Range, getRange1)
	#rospy.Subscriber("/ultrasound2", Range, getRange2)
	rospy.Subscriber("/turtle1/cmd_vel", Twist, getTeleop)

def getIMU(data):
	print(data);

def getRC(data):
	print(data.channels)

def getRange1(data):
	global rangeUltrasonicFront
	rangeUltrasonicFront = data.range

def getRange2(data):
	global rangeUltrasonicBack
	rangeUltrasonicBack = data.range

def getTeleop(data):
	setRCOverrideTeleop(data.linear.x, data.angular.z)

def constraintRC(channel, min, max):
	global RCOverride

	if(RCOverride[channel]<min):
		RCOverride[channel]=min

	if(RCOverride[channel]>max):
		RCOverride[channel]=max

def setRCOverrideTeleop(throttleInput, steeringInput):
	global RCOverride
	pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
	rate = rospy.Rate(10)
	flag = True
	prevThrottleChannel = RCOverride[ThrottleChannel]
	prevSteeringChannel = RCOverride[SteeringChannel]
	RCOverride[ThrottleChannel] = prevThrottleChannel - int(throttleInput)*throttleMultPWM
	RCOverride[SteeringChannel] = prevSteeringChannel - int(steeringInput)*steeringMultPWM

	constraintRC(ThrottleChannel, throttleMinPWM, throttleMaxPWM)
	constraintRC(SteeringChannel, steeringMinPWM, steeringMaxPWM)

	#if obstacle is too close, avoid forward/backward movement
	if(rangeUltrasonicFront < 0.6):
		if(RCOverride[ThrottleChannel] < 1500):
			RCOverride[ThrottleChannel] = 1500

	if(rangeUltrasonicBack < 0.6):
		if(RCOverride[ThrottleChannel] > 1500):
			RCOverride[ThrottleChannel] = 1500

	while not rospy.is_shutdown() and flag:
		flag = False
		pub.publish(RCOverride)
		#pub.publish(RCOverride)
		rate.sleep()

def setRCArm():
	pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
	RCOverride[ThrottleChannel] = 1000
	pub.publish(RCOverride)

if __name__ == '__main__':
	#try:
		print("Program starts")
		rospy.init_node('semiAutoControl', anonymous=True)
		#setRCArm()
		arm()
		#time.sleep(3)
		subscribeToData()
		rospy.spin() #prevent from exiting


	#except rospy.ROSInterrupException:
	#	pass
