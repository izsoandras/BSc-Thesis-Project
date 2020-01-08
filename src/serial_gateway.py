#!/usr/bin/env python
import rospy
import serial
import yaml
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from std_msgs.msg import Empty

# This node handles all the serial communication

class SerialGateway:
    def __init__(self, serialport, baud=115200):
        self.ser = serial.Serial(port=serialport, baudrate=baud, writeTimeout=0)
        self.pwm_freq = 1000
        # Perform subscriptions and adverising
        self.cmd_velSub = rospy.Subscriber('/robopro/cmd_vel', Twist, self.cmd_velCallback)
        self.pwm_freq_incSub = rospy.Subscriber('/robopro/setstatus/pwm_freq_inc', Empty, self.pwm_freq_incCallback)
        self.pwm_freq_decSub = rospy.Subscriber('/robopro/setstatus/pwm_freq_dec', Empty, self.pwm_freq_decCallback)
        self.rstSub = rospy.Subscriber('/robopro/setstatus/reset', Empty, self.rstCallback)
        self.calibrateSub = rospy.Subscriber('/robopro/setstatus/calibrate', Empty, self.calibrateCallback)
        self.inercPosePub = rospy.Publisher('/robopro/inercPose', Pose)
        self.encPosePub = rospy.Publisher('/robopro/encPose', Pose)
        self.pwm_freqPub = rospy.Publisher('/robopro/status/pwm_freq', Int16)

    # Writes line to the serial and logs the message
    def writeLineSerial(self, string):
        ser_msg = string + '\n'
        self.ser.write(str.encode(ser_msg))
        rospy.loginfo("Sending serial message: " + ser_msg)

    # Handles movement commands
    def cmd_velCallback(self, msg):
        ser_msg = "S:{0},{1}".format(msg.linear.x, msg.linear.y)
        self.writeLineSerial(ser_msg)

    # Sends a message to change the PWM frequency to the given parameter
    def changePWM(self, pwm_freq):
        self.pwm_freq = pwm_freq
        ser_msg = "F:{0}".format(self.pwm_freq)
        self.writeLineSerial(ser_msg)
        self.pwm_freqPub.publish(self.pwm_freq)

    # Callback for PWM decrease message
    def pwm_freq_decCallback(self, msg):
        self.changePWM(max(0, self.pwm_freq - 100))


    # Callback for PWM increase message
    def pwm_freq_incCallback(self, msg):
        self.changePWM(min(20000, self.pwm_freq + 100))

    # Callback for reset message
    def rstCallback(self, msg):
        self.writeLineSerial("R:")

    # Callback for change to magnetometercalibration mode message
    def calibrateCallback(self, msg):
        self.writeLineSerial("C:")

    # Check serial buffer if new message has arrived
    def checkSerial(self, event):
        line = self.ser.readline()
        # rospy.loginfo(line)
        splitted = line.split()

        # Interpret the message and forward it to the ROS system
        if len(splitted) == 6:
            try:
                inercMsg = Pose()
                inercMsg.position.x = float(splitted[1])
                inercMsg.position.y = float(splitted[2])
                inercMsg.orientation.z = float(splitted[0])
                self.inercPosePub.publish(inercMsg)
                encMsg = Pose()
                encMsg.position.x = float(splitted[4])
                encMsg.position.y = float(splitted[5])
                encMsg.orientation.z = float(splitted[3])
                self.encPosePub.publish(encMsg)
            except ValueError:
                rospy.loginfo("Could not parse incoming serial message")
                return


if __name__ == "__main__":
    rospy.init_node("robopro_serial_gateway")
    gtwy = SerialGateway("/dev/ttyAMA0", 115200)
    with open(r'/home/pi/catkin_ws/src/robopro/config.yaml') as conf:
        config = yaml.load(conf)
        gtwy.changePWM(config["PWMfreq"])

    rospy.Timer(rospy.Duration(0.01), gtwy.checkSerial)
    rospy.spin()
