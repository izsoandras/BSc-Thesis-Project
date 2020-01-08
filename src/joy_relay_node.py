#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Empty

# Node to convert the built-in joy node's messages to messages of Robopro
class JoyRelay():
    def __init__(self):
        # Store the buttons' statuses
        self.button5 = False    # "Shift down" - Decrease PWM frequency
        self.button6 = False    # "Shift up" - Increase PWM frequency
        self.button2 = False    # Reset microcontrollers position state
        self.button4 = False    # Put microcontroller to magnetometer calibration mode
        # Subscribe to the joy node
        self.sub = rospy.Subscriber('joy', Joy, self.callback_func)
        # Publish movement control commands
        self.cmd_velPub = rospy.Publisher('/robopro/cmd_vel', Twist)
        # Publish PWM frequency commands
        self.pwm_freq_incPub = rospy.Publisher('/robopro/setstatus/pwm_freq_inc', Empty)
        self.pwm_freq_decPub = rospy.Publisher('/robopro/setstatus/pwm_freq_dec', Empty)
        # Publish reset message
        self.rstPub = rospy.Publisher('/robopro/setstatus/reset', Empty)
        # Publish mode change to magnetometer calibration mode
        self.calibratePub = rospy.Publisher('/robopro/setstatus/calibrate', Empty)

    # Handle received joystick message
    def callback_func(self, msg):
        # Prepare and publish movement control message
        rp_msg = Twist()
        rp_msg.linear.x = pow(msg.axes[1], 3) * 100
        rp_msg.linear.y = pow(msg.axes[3], 3) * 100
        rp_msg.linear.z = 0
        rp_msg.angular.x = 0
        rp_msg.angular.y = 0
        rp_msg.angular.z = 0
        self.cmd_velPub.publish(rp_msg)
        # Check button statuses, send commands accordingly
        if not self.button2 and msg.buttons[1] == 1:
            self.rstPub.publish()

        if not self.button5 and msg.buttons[4] == 1:
            self.pwm_freq_decPub.publish()

        if not self.button6 and msg.buttons[5] == 1:
            self.pwm_freq_incPub.publish()

        if not self.button4 and msg.buttons[3] == 1:
        	self.calibratePub.publish()

        # Update button statuses
        self.button2 = msg.buttons[1]
        self.button4 = msg.buttons[3]
        self.button5 = msg.buttons[4]
        self.button6 = msg.buttons[5]


if __name__ == "__main__":
    rospy.init_node("joystick2serial")
    JoyRelay()
    rospy.spin()
