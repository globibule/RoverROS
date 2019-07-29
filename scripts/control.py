#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

import tf
import math
import time

exec_time = 1/25;

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

def subsIMU():
	rospy.Subscriber("/mavros/imu/data", Imu, getIMU)
	rospy.spin()

def subsRCIN():
	rospy.Subscriber("/mavros/rc/in", RCIn, getRC)
	rospy.spin()

def subsUltrasonic():

def getIMU(data):
	print(data);

def getRC(data):
	#pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    	#rate = rospy.Rate(10) # 10hz
	print(data.channels)

if __name__ == '__main__':
	#try:
		print("Program starts")
		rospy.init_node('semiAutoControl', anonymous=True)
		#arm()
		#time.sleep(3)
		subsRCIN()

	#except rospy.ROSInterrupException:
	#	pass
