#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
import time
import Adafruit_PCA9685

# Import the PCA9685 module.

pwm = Adafruit_PCA9685.PCA9685()
# Configure min and max servo pulse lengths

tilt_init=1330
tilt_range=200

pan_init=1250
pan_range=600

fire_init=1700
fire_push=1000


# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 50 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    pwm.set_pwm(0, 0, servo_min)
    time.sleep(1)
    pwm.set_pwm(0, 0, servo_max)
    time.sleep(1)






kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(650, 1850)
kit.servo[1].set_pulse_width_range(1130, 1530)
kit.servo[2].set_pulse_width_range(1000, 1700)

kit.servo[0].actuation_range = 100
kit.servo[1].actuation_range = 100
kit.servo[2].actuation_range = 100



time.sleep(1)

kit.servo[0].angle = 180

def sign(x):
	if (x < 0):
		return -1
	if (x > 0):
	 	return 1
 	else:
	 return 0


def joy_callback(joydata):


	#left
    if ((joydata.axes[0]) != 0):
        kit.servo[0].actuation_range += 5 * sign(joydata.axes[0])

    if ((joydata.axes[1]) != 0):
        kit.servo[2].actuation_range += 5 * sign(joydata.axes[1])

    if ((joydata.buttons[1]) == 1):
        kit.servo[2].actuation_range = 0

    if ((joydata.buttons[1]) == 0):
    	kit.servo[2].actuation_range = 100

def tetra_utility_loop():
    rospy.init_node('ros_turret', disable_signals=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    kit.servo[0].angle = 50
    kit.servo[1].angle = 50
    kit.servo[2].angle = 100
    

    r = rospy.Rate(500)  # 500hz

    while not rospy.is_shutdown():
        r.sleep()
    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass