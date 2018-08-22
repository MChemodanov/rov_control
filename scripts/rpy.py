#!/usr/bin/env python

import rospy
import math
import operator

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

class RpyConvertor:
    def __init__ (self):
        rospy.init_node('surface_side', anonymous=True)
	self.pub_roll  = rospy.Publisher('rpy/roll', Float32, queue_size=10)
	self.pub_pitch = rospy.Publisher('rpy/pitch', Float32, queue_size=10)
	self.pub_yaw   = rospy.Publisher('rpy/yaw', Float32, queue_size=10)

        rospy.Subscriber("/imu", Imu, self.callback)
        rospy.Subscriber("/calibrate_imu", Int32, self.callback_calibrate)
        self.calibrate_data = [0, 0, 0]
        self.last_data = [0, 0, 0]

    def calculate(self, data):
        tmp = data.orientation
        tmp = [tmp.x, tmp.y, tmp.z, tmp.w]
        result = euler_from_quaternion(tmp)
        result = map(math.degrees, result)
        return result

    def callback_calibrate(self, data):
        self.calibrate_data = self.last_data

    def callback(self, data):
        self.last_data = self.calculate(data)
        result = map(operator.sub, self.last_data, self.calibrate_data) 
        self.pub_roll.publish(result[1]) 
        self.pub_pitch.publish(result[0]) 
        self.pub_yaw.publish(result[2]) 

    def loop(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    core = RpyConvertor()
    core.loop()
