#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float32
from rov_control.msg import Servo
from sensor_msgs.msg import Joy
from akara_msgs.msg import Thruster
from sensor_msgs.msg import FluidPressure

DEADBAND = 0.005

CAMERA_AXIS = 6
FORWARD_AXIS = 1
UP_AXIS = 2
ROTATE_AXIS = 3
LAG_AXIS = 4 #0
TURN_AXIS = 3
DEPTH_AXIS = 2

CAMERA_CH = 7

DEPTH_K = 0.01

DEPTH_SHIFT = 0

STOP_BUTTON = 1

ESP_MAX = 375
MAX_AMP = 0.25 #0.25
ESC_AMP = ESP_MAX*MAX_AMP
FORWARD_K = 0.8 #2.375

MODE = "WORK" # "ROLL_PID" "PITCH_PID" "PITCH_K" "WORK"

THRUSTER_UP_BACK = 5
THRUSTER_ROTATE = 2
THRUSTER_UP_RIGHT = 4
THRUSTER_FWD_LEFT  = 1
THRUSTER_FWD_RIGHT = 3
THRUSTER_UP_LEFT = 6



def trunc_value(value, low_border = -1, high_border = 1):
    if low_border > high_border:
        l = high_border
        h = low_border
    else:
        l = low_border
        h = high_border
    if value > h:
        value = h
    elif value < l:
        value = l
    return value

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class KeepValueAxis:
    def __init__ (self, min_in, max_in, min_out, max_out, shift, default, step):
        # self.publisher = publisher
        self.min_in = min_in
        self.max_in = max_in
        self.min_out = min_out
        self.max_out = max_out
        self.shift = shift
        self.default = default
        self.raw_value = (max_in-min_in)/2
        self.set_default()
        self.step = step

    def scale(self, value):
        self.raw_value = trunc_value(value, self.min_in, self.max_in)
        self.scaled_value = remap(self.raw_value, self.min_in, self.max_in, self.min_out, self.max_out) + self.shift
    
    def set_default(self):
        self.scaled_value = self.default

    def set_value(self, value):
        self.scale(value)
        #rospy.logwarn(self.raw_value)
        #rospy.logwarn(self.scaled_value)
        return self.scaled_value

    def do_step(self, count):
        self.set_value(self.raw_value + count*self.step)

    def shift_up(self):
        self.shift += self.step

    def shift_down(self):
        self.shift -= self.step

        
class BrThruster:
    STOP_LOW = 1475
    STOP_HIGH = 1525

    def __init__(self, reversed = False, min_amp = ESC_AMP, max_amp = ESC_AMP):
        self.set_amp(min_amp, max_amp)
        self.reversed = reversed
        self.controller_fwd = KeepValueAxis(0, 1, self.STOP_HIGH, self.STOP_HIGH + max_amp, 0, 1500, 0)
        self.controller_rvr = KeepValueAxis(0, -1, self.STOP_LOW, self.STOP_LOW - min_amp, 0, 1500, 0)

        self.set_default()


    def set_amp(self, min_amp, max_amp):
        self.min_amp = min_amp
        self.max_amp = max_amp
       

    def set_default(self):
        self.controller_fwd.set_default()
        self.scaled_value = self.controller_fwd.scaled_value
        return self.scaled_value

    def set_value(self, value):
        if self.reversed:
            value *= -1
        if abs(value) > DEADBAND:
            #rospy.logwarn(value)
            if value > 0:
               self.scaled_value = self.controller_fwd.set_value(value)
            else:
               self.scaled_value = self.controller_rvr.set_value(value)
            #srospy.logwarn(self.scaled_value)
        else:
            self.set_default()
        return self.scaled_value
        
                    #self.channels[THRUSTER_UP_LEFT].set_value(fwd)
            #self.channels[THRUSTER_UP_RIGHT].set_value(fwd)


class RovController:
    def __init__ (self):
        rospy.init_node('surface_side', anonymous=True)
        self.publisher = rospy.Publisher('servos', Servo, queue_size=1)
        self.channels = [None]*8
        self.channels[CAMERA_CH] = KeepValueAxis(0, 180, 500, 2300, 0, 900, 5)
        self.led = KeepValueAxis(0, 1, 0, 1, 0, 0, 0.05)

        rospy.logwarn("!")
        self.channels[THRUSTER_UP_LEFT] = BrThruster(True, ESP_MAX, ESP_MAX)
        self.channels[THRUSTER_UP_RIGHT] = BrThruster(False, ESP_MAX, ESP_MAX)
        rospy.logwarn("!")
        self.channels[THRUSTER_UP_BACK] = BrThruster(False)
        self.channels[THRUSTER_ROTATE] = BrThruster(False, ESC_AMP/MAX_AMP*0.2, ESC_AMP/MAX_AMP*0.5) #cw, cr cw
        #self.channels[THRUSTER_ROTATE] = BrThruster(False, MAX_AMP, ESP_MAX)  # cw, cr cw

        self.channels[THRUSTER_FWD_LEFT] = BrThruster(False, ESP_MAX, ESP_MAX)
        self.channels[THRUSTER_FWD_RIGHT] = BrThruster(True, ESP_MAX, ESP_MAX)

        rospy.Subscriber("joy", Joy, self.callback)
        rospy.Subscriber("rpy/roll", Float32, self.callback_roll)
        rospy.Subscriber("rpy/pitch", Float32, self.callback_pitch)
        rospy.Subscriber("rpy/yaw", Float32, self.callback_yaw)
        rospy.Subscriber("pressure", FluidPressure, self.callback_pressure)
        rospy.Subscriber("calibrate_imu", Float32, self.callback_calibrate)

        self.pub_td = rospy.Publisher("target_depth", Float32, queue_size=10)
        self.pub_ty = rospy.Publisher("target_yaw", Float32, queue_size=10)
        self.pub_depth = rospy.Publisher("depth", Float32, queue_size=10)
        self.pub_led = rospy.Publisher("led", Float32, queue_size=10)
        self.pub_manipulator = rospy.Publisher("manipulator", Thruster, queue_size=10)

        self.data = None
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.target_yaw = 0

        self.pitch_int = 0
        self.roll_int = 0
        self.pid_pitch_p = 0.001
        self.pid_pitch_d = 0.005
        self.pid_roll_p = 0.002
        self.previous_pitch = 0
        self.pid_depth_p = 0
        self.depth = 0
        self.target_depth = 0

        self.enabled = False

        self.depth_scaler = KeepValueAxis(-1, 1, 0, 2, 0, 0, 0.01)

    def parse_buttons(self, message):
        self.channels[CAMERA_CH].do_step(message.axes[CAMERA_AXIS])
        if message.buttons[4]:
            self.depth_scaler.shift_up()
        elif message.buttons[6]:
            self.depth_scaler.shift_down()

        if (message.axes[4]):
            self.led.do_step(-1*message.axes[4])
            self.pub_led.publish(self.led.scaled_value)
        if message.axes[5]:
            out = Thruster()
            out.power = [message.axes[5]*105]
            self.pub_manipulator.publish(out)

    def parse_thrusters(self, message):
        lag = message.axes[LAG_AXIS]*0.2
        fwd = message.axes[FORWARD_AXIS]
        depth = message.axes[DEPTH_AXIS]
        rotate = message.axes[ROTATE_AXIS]
        self.target_depth = self.depth_scaler.set_value(depth)
        self.pub_td.publish(self.target_depth)
        if MODE == "WORK":
            self.target_depth = self.depth_scaler.set_value(depth)
            depth_power = depth # s(self.depth - self.target_depth)*self.pid_depth_p
            roll_delta  = self.roll_int*0.01
            self.pitch_int += self.pitch*0.005

            if lag == 0:
                self.roll_int += self.roll * 0.01


            pitch_delta  = self.pitch*0.01 #+ self.pitch_int*0.02 #+ self.pitch_diff

            #self.target_yaw = self.yaw
            if rotate != 0 and message.buttons[0]:
                self.target_yaw = self.yaw
                self.pitch_int = 0

            self.pub_ty.publish(self.target_yaw)

            yaw_delta = self.target_yaw - self.yaw
            if yaw_delta > 180:
                yaw_delta -= 180
            elif yaw_delta < -180:
                yaw_delta += 180

            if abs(yaw_delta) > 20:
                yaw_delta = math.copysign(20, yaw_delta)
            yaw_p = 0.01
            #if lag != 0:
            #    yaw_p = 0.01
            #    self.roll_delta = 0
            #else:
            #    yaw_p = 0
            #lag = 0

            #rospy.logwarn("Delta: %f" % (yaw_delta) )


            roll_delta = 0
            if message.buttons[3]:
                if message.buttons[0]:
                    self.channels[THRUSTER_UP_LEFT].set_value(lag * 0.75 + fwd * 0.1)
                    self.channels[THRUSTER_UP_RIGHT].set_value(-lag * 0.75 + fwd * 0.1)
                    self.channels[THRUSTER_UP_BACK].set_value(-fwd * 0.1)
                else:
                    self.channels[THRUSTER_UP_LEFT].set_value(lag * 2 + fwd * 0.5)
                    self.channels[THRUSTER_UP_RIGHT].set_value(-lag * 2 + fwd * 0.5)
                    self.channels[THRUSTER_UP_BACK].set_value(-fwd * 0.5)

                self.channels[THRUSTER_ROTATE].set_value(0)
                self.channels[THRUSTER_FWD_LEFT].set_value(depth_power*1)
                self.channels[THRUSTER_FWD_RIGHT].set_value(depth_power*1)
            else:
                if message.buttons[8]:
                  self.channels[THRUSTER_FWD_LEFT].set_value((fwd * 0.5 - rotate * 0.2 - yaw_delta * yaw_p)*0.5)
                  self.channels[THRUSTER_FWD_RIGHT].set_value((fwd * 0.5 + rotate * 0.2 + yaw_delta * yaw_p)*0.5)
                else:
                  self.channels[THRUSTER_FWD_LEFT].set_value((fwd * 0.5 - rotate * 0.2 - yaw_delta * yaw_p)*0.25)
                  self.channels[THRUSTER_FWD_RIGHT].set_value((fwd * 0.5 + rotate * 0.2 + yaw_delta * yaw_p)*0.25)

                self.channels[THRUSTER_UP_BACK].set_value(trunc_value(depth_power + pitch_delta))

                self.channels[THRUSTER_ROTATE].set_value(yaw_delta * yaw_p * 2 + rotate)
                self.channels[THRUSTER_UP_LEFT].set_value(trunc_value(depth_power - roll_delta - pitch_delta - lag*1.5)/FORWARD_K*0.1)
                self.channels[THRUSTER_UP_RIGHT].set_value(trunc_value(depth_power + roll_delta  - pitch_delta + lag*1.5)/FORWARD_K*0.1)

        elif MODE == "PITCH_K":
            k =  1.35 + abs(depth)*0.1
            rospy.logwarn(k)
            rospy.logwarn(fwd)
            self.channels[THRUSTER_UP_LEFT].set_value(fwd/k)
            self.channels[THRUSTER_UP_RIGHT].set_value(fwd/k)
            self.channels[THRUSTER_UP_BACK].set_value(fwd)

        elif MODE == "PITCH_PID":
            self.pid_pitch_p = self.target_depth*0.01
            rospy.logwarn(self.pitch)
            rospy.logwarn(self.pid_pitch_p)
            rospy.logwarn(fwd)

            depth_power = fwd

            roll_delta  = -1*self.roll*0.02  # self.pid_roll_p
            pitch_delta  = 0 #self.pitch*self.pid_pitch_p

            self.channels[THRUSTER_UP_LEFT].set_value(trunc_value(depth_power - roll_delta - pitch_delta)*FORWARD_K)
            self.channels[THRUSTER_UP_RIGHT].set_value(trunc_value(depth_power + roll_delta  - pitch_delta)*FORWARD_K)
            self.channels[THRUSTER_UP_BACK].set_value(trunc_value(depth_power  + pitch_delta))

        elif MODE == "ROLL_PID":
            self.pid_roll_p = self.target_depth
            rospy.logwarn(self.pid_roll_p)
            rospy.logwarn(fwd)

            depth_power = fwd

            roll_delta  = self.roll*self.pid_roll_p
            pitch_delta  = self.pitch*self.pid_pitch_p + (self.pitch - self.previous_pitch)*self.pid_pitch_d


            self.channels[THRUSTER_UP_LEFT].set_value(trunc_value(depth_power - roll_delta - pitch_delta))
            self.channels[THRUSTER_UP_RIGHT].set_value(trunc_value(depth_power + roll_delta  - pitch_delta))
            self.channels[THRUSTER_UP_BACK].set_value(trunc_value(depth_power  + pitch_delta))
            self.previous_pitch = self.pitch

        else:
            rospy.logwarn(fwd)
            self.channels[THRUSTER_UP_BACK].set_value(fwd)
            #self.channels[THRUSTER_ROTATE].set_value(fwd)
            #self.channels[THRUSTER_FWD_LEFT].set_value(fwd)
            #self.channels[THRUSTER_FWD_RIGHT].set_value(fwd)
            #self.channels[THRUSTER_UP_LEFT].set_value(fwd)
            #self.channels[THRUSTER_UP_RIGHT].set_value(fwd)
            #"""

    def callback(self, data):
        self.data = data

    def callback_pitch(self, pitch):
        self.pitch = float(pitch.data)

    def callback_roll(self, roll):
        self.roll = float(roll.data)

    def callback_yaw(self, yaw):
        self.yaw = float(yaw.data)

    def callback_pressure(self, depth):
        self.pressure = float(depth.fluid_pressure)
        self.depth = DEPTH_K*self.pressure + DEPTH_SHIFT
        self.pub_depth.publish(self.depth)

    def callback_calibrate(self, data):
        DEPTH_SHIFT = -self.depth

    def apply_data(self):
        if self.data.buttons[STOP_BUTTON]:
            self.channels[THRUSTER_UP_LEFT].set_default()
            self.channels[THRUSTER_UP_RIGHT].set_default()
            self.channels[THRUSTER_UP_BACK].set_default()
            self.channels[THRUSTER_ROTATE].set_default()
            self.channels[THRUSTER_FWD_LEFT].set_default()
            self.channels[THRUSTER_FWD_RIGHT].set_default()

            self.channels[CAMERA_CH].set_value(90)
            self.target_yaw = self.yaw
        else:
            self.parse_buttons(self.data)
            self.parse_thrusters(self.data)
        
        out = Servo()
        for i in range(8):
            if not self.channels[i] is None:
                out.channels[i] = self.channels[i].scaled_value   
            else:
                out.channels[i] = 0

        self.publisher.publish(out)

    def loop(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if self.data is not None:
                self.apply_data()
            rate.sleep()

if __name__ == '__main__':
    core = RovController()
    core.loop()
