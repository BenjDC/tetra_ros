#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import rospy
#from sensor_msgs.msg import Joy
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(650, 1850)
kit.servo[1].set_pulse_width_range(1130, 1530)
kit.servo[2].set_pulse_width_range(1000, 1700)

kit.servo[0].actuation_range = 100
kit.servo[1].actuation_range = 100
kit.servo[2].actuation_range = 100

kit.servo[0].angle = 50

time.sleep(1)

kit.servo[0].angle = 180