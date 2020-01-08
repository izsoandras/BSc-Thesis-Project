#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import RPi.GPIO as GPIO
import time

# Program for follow a (red) ball
# Heavyweight, should be split up into more nodes in the future

# DEBUG values
SET_GUI = False             # Do or do not show GUI
DEBUG_MOTORSPEED = False    # Do or do not write motor speed commands on console
DEBUG_TIMING = False        # Do or do not write how much time each processing step takes on console
DEBUG_CIRCLEPOS = False      # Do or do not write detected circle position on console

def signum(a):
    return -1 if a < 0 else 1




class ImageDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/rec_image/compressed",CompressedImage,queue_size=1)
        self.image_sub = rospy.Subscriber("/robocam/image_raw/compressed",CompressedImage,self.callback)
        self.pub = rospy.Publisher('/robopro/cmd_vel', Twist, queue_size=1)
        self.main_window = "Camera image"
        # detection variables
        self.posX = None                 # X position
        self.posX_prev = 0               # X position in the previous iteration
        self.posY = None                 # Y position
        self.posX_exp_filter_coeff = 0.8 # The amount of how much the current measurement changes the position. [0,1]. current = alpha * measurement + (1-alpha) * previous
        self.radius = None               # Circle radius
        self.radius_prev = 0             # Previous circle radius
        self.rad_exp_filter_coeff = 0.8  # The amount of how much the current measurement changes the radius. [0,1]. current = alpha * measurement + (1-alpha) * previous
        self.speed = 0                   # Speed to send to the motor controller
        self.angleCorr = 0               # The difference between the two tracks so the robot turns
        self.roi = None                  # Part of the image where we expect to find the ball
        self.last_circle_time = 0        # Time elapsed since circle was last detected
        self.last_movement_time = time.time()    # Time elapsed since last movement
        self.gpioFrame = False

        # PID control variables
        self.distRef = 20                # Reference distance
        self.distErr = 0                 # Difference between the actual distance of the ball and the ref
        self.angleErr = 0                # Difference between the actual angle and the ref (= 0)
        self.distIntegral = 0            # Integral of the distance error for calculating the I part
        self.angleIntegral = 0           # Integral of the angle error for calculating the I part
        self.distDeriv = 0               # Derivative of the distance error for calculating the D part
        self.angleDeriv = 0              # Derivative of the angle error for calculating the D part
        self.Kp_dist = 2                 # Coefficient of the P part of the speed PID
        self.Ki_dist = 0                 # Coefficient of the I part of the speed PID
        self.Kd_dist = 0                 # Coefficient of the D part of the speed PID
        self.Kp_angle = 0.1              # Coefficient of the P part of the angle PID
        self.Ki_angle = 8                # Coefficient of the I part of the angle PID
        self.Kd_angle = 0                # Coefficient of the D part of the angle PID

        # red filter values
        self.red_lower_max_h = 6         # hue -> color
        self.red_upper_min_h = 159       # hue -> color
        self.red_min_s = 100             # saturation -> white - colorful
        self.red_min_v = 100             # value -> black - birght

            



    # Change the global values according to the trackbars
    def on_trackbar_min_s(self,val):
        self.red_min_s = val


    def on_trackbar_min_v(self,val):
        self.red_min_v = val


    def on_trackbar_lower_max_h(self,val):
        self.red_lower_max_h = val


    def on_trackbar_upper_min_h(self,val):
        self.red_upper_min_h = val


    def send_command(self, left, right):
        rospy.loginfo("Left: %d\tRight: %d",left,right)
        rp_msg = Twist()
        rp_msg.linear.x = left
        rp_msg.linear.y = right
        rp_msg.linear.z = 0
        rp_msg.angular.x = 0
        rp_msg.angular.y = 0
        rp_msg.angular.z = 0
        self.pub.publish(rp_msg)
        


    def callback(self,msg):
        # remember the time for profiling
        if DEBUG_TIMING:
            timings = {"total_time": time.time()}
            now_time = time.time()

        # read the image from the camera
            # uncompressed
        #arr = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
            # Uncomment following two lines for CompressedImage topic
        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, channels = image.shape

        self.gpioFrame = not self.gpioFrame
        GPIO.output(19, GPIO.HIGH if self.gpioFrame else GPIO.LOW)
        roi = None

        if DEBUG_TIMING:
            timings["camera.read"] = time.time() - now_time
            now_time = time.time()

        
        # switch to HSV colorspace for easier color filter
        hsv = cv2.cvtColor(np.uint8(image), cv2.COLOR_BGR2HSV)

        if DEBUG_TIMING:
            timings["cv2.cvtColor"] = time.time() - now_time
            now_time = time.time()


        # filter the 2 red parts of colorspace
        mask_lower = cv2.inRange(hsv, np.array([0, self.red_min_s, self.red_min_v]), np.array([self.red_lower_max_h, 255, 255]))
        mask_upper = cv2.inRange(hsv, np.array([self.red_upper_min_h, self.red_min_s, self.red_min_v]), np.array([179, 255, 255]))

        if DEBUG_TIMING:
            timings["2x cv2.inRange"] = time.time() - now_time
            now_time = time.time()

        # unite the two red parts
        mask_red = cv2.addWeighted(mask_upper, 1, mask_lower, 1, 0)

        if DEBUG_TIMING:
            timings["cv2.addWeighted"] = time.time() - now_time
            now_time = time.time()

        # median blur for better recognition
        mask_red = cv2.medianBlur(mask_red, 3)

        if DEBUG_TIMING:
            timings["cv2.medianBlur"] = time.time() - now_time
            now_time = time.time()

        # erosion and dilation - removed for extra speed
        # kernel = np.ones((9, 9), np.uint8)
        # mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        # mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        # find circles
        circles = cv2.HoughCircles(mask_red, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=10, minRadius=7, maxRadius=320)

        if DEBUG_TIMING:
            timings["cv2.HoughCircles (red)"] = time.time() - now_time
            now_time = time.time()

        # control the robot according to the measures
        if circles is not None:  # or time.time() - last_circle_time < 3:
            if circles is not None:
                self.last_circle_time = time.time()
                circles = np.uint16(np.around(circles))
                circle = circles[0, 0]
                self.posX = circle[0]
                self.posY = circle[1]
                self.radius = circle[2]
                if DEBUG_CIRCLEPOS:
                    rospy.loginfo("CIRCLE_POS> X:{}\tY: {}\tr: {}\n".format(self.posX, self.posY, self.radius))

                xLower = max(self.posX - int(1.5 * self.radius), 0)
                xHigher = min(self.posX + int(1.5 * self.radius), width - 1)
                yLower = max(self.posY - int(1.5 * self.radius), 0)
                yHigher = min(self.posY + int(1.5 * self.radius), height - 1)
                roi = cv2.cvtColor(image[yLower: yHigher,
                                   xLower: xHigher], cv2.COLOR_BGR2GRAY)
    
                if DEBUG_TIMING:
                    timings["ROI"] = time.time() - now_time
                    now_time = time.time()

                if SET_GUI:
                    # draw the outer circle
                    cv2.circle(image, (self.posX, self.posY), self.radius, (0, 0, 0), 2)
                    # draw the center of the circle
                    cv2.circle(image, (self.posX, self.posY), 2, (0, 0, 0), 3)

                if roi is not None:
                    graycircles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=80,
                                                   minRadius=int(0.7 * self.radius), maxRadius=2 * self.radius)

                    if DEBUG_TIMING:
                        timings["cv2.HoughCircles (grayscale)"] = time.time() - now_time
                        now_time = time.time()

                    if graycircles is not None:
                        circle = graycircles[0, 0]
                        self.posX = xLower + circle[0]
                        self.posY = yLower + circle[1]
                        self.radius = circle[2]
                        if DEBUG_CIRCLEPOS:
                            rospy.loginfo("ACC_CIRC_POS> X: {}\tY: {}\tr: {}\t".format(posX, posY, radius))

                GPIO.output(26, GPIO.HIGH)

                # exp avg for noise reduction
                self.radius = self.rad_exp_filter_coeff * self.radius + (1-self.rad_exp_filter_coeff) * self.radius_prev
                self.posX = self.posX_exp_filter_coeff * self.posX + (1-self.posX_exp_filter_coeff) * self.posX_prev

                # calculate PID parameters
                self.distErr = self.distRef - self.radius
                self.distIntegral += self.distErr
                self.angleErr = width / 2 - self.posX
                self.angleIntegral += self.angleErr

                # calculate control values
                self.speed = min(self.Kp_dist * self.distErr, 100)
                self.angleCorr = self.Kp_angle * self.angleErr
            else:
                GPIO.output(26, GPIO.LOW)

            # control
            self.send_command(self.speed + self.angleCorr, self.speed - self.angleCorr)
            self.last_movement_time = time.time()

            # save for next iteration
            self.radius_prev = self.radius
            self.posX_prev = self.posX

            if DEBUG_TIMING:
                timings["motor control"] = time.time() - now_time
                now_time = time.time()

            if SET_GUI:
                # draw the outer circle
                cv2.circle(image, (int(self.posX), int(self.posY)), int(self.radius), (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(image, (int(self.posX), int(self.posY)), 2, (0, 255, 0), 3)

            # if graycircles is not None:
            #    cv2.circle(gray, (j[0], j[1]), j[2], (0, 255, 0), 2)
            #    cv2.circle(gray, (j[0], j[1]), 2, (0, 255, 0), 3)

            # if there is no circle on image
        else:
            GPIO.output(26, GPIO.LOW)
            if 5 < time.time() - self.last_movement_time < 12:
                self.send_command(signum(self.angleErr) * 40, signum(self.angleErr) * -40)
                if DEBUG_MOTORSPEED:
                    rospy.loginfo("S:{},{}".format(signum(self.angleErr) * 40, signum(self.angleErr) * -40))
            else:
                self.send_command(0,0)
                self.posX = None
                self.posY = None
                self.radius = None
                if DEBUG_TIMING:
                    timings["motor stop"] = time.time() - now_time
                    now_time = time.time()

        if DEBUG_TIMING:
            timings["total_time"] = time.time() - timings["total_time"]
            rospy.loginfo("Timings: " + '\t'.join(["%s= %fms"%(k,v*1000.0) for (k,v) in timings.items()]))

        
        # send image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.image_pub.publish(msg)
        
        if SET_GUI:
            # show the frame
            cv2.namedWindow(self.main_window)
            cv2.createTrackbar("Upper red min H:", self.main_window, 0, 179, self.on_trackbar_upper_min_h)
            cv2.setTrackbarPos("Upper red min H:", self.main_window, self.red_upper_min_h)
            cv2.createTrackbar("Lower red max H:", self.main_window, 0, 179, self.on_trackbar_lower_max_h)
            cv2.setTrackbarPos("Lower red max H:", self.main_window, self.red_lower_max_h)
            cv2.createTrackbar("Red min S:", self.main_window, 0, 255, self.on_trackbar_min_s)
            cv2.setTrackbarPos("Red min S:", self.main_window, self.red_min_s)
            cv2.createTrackbar("Red min V:", self.main_window, 0, 255, self.on_trackbar_min_v)
            cv2.setTrackbarPos("Red min V:", self.main_window, self.red_min_v)
            cv2.imshow(self.main_window, image)
            cv2.imshow("HSV", hsv)
            cv2.imshow("Red", mask_red)
            if roi is not None:
                cv2.imshow("roi", roi)

            
            cv2.waitKey(1)



if __name__ == "__main__":
    # set up the used GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(26, GPIO.OUT)    # LED showing if circle is detected
    GPIO.output(26, GPIO.LOW)
    GPIO.setup(19, GPIO.OUT)    # LED showing framerate (gets inverted every frame)
    GPIO.output(19, GPIO.LOW)
    ic = ImageDisplay()
    rospy.init_node("imagedisplay", anonymous = True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
