#!/usr/bin/env python
import time
import math
import rospy
import Tkinter
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16

# Node to visualize the position estimation of the different algorithms
# Note that ORB-SLAM2 is not fully integrated
# Magnetometer algorithms produce absolute heading, computer vision algorithms produce realitve and is not corrected to absolute
#
#    N
#    ^
#    |
# W<- ->E
#    |
#    v
#    S


# Window attributes
WIDTH = 800                        
HEIGHT = 800                       
RADIUS = 10
SMALL_RADIUS = 1
PIXEL_PER_METER = 100              
PIXEL_PER_DM = PIXEL_PER_METER / 10

# Prepare drawing area
top = Tkinter.Tk()                
top.title("Robot pos visualizer")
C = Tkinter.Canvas(top, bg="black", height=HEIGHT, width=WIDTH)

# Create markers for different algorithms
imuMarker = None    # Accelerometer - Magnetometer position
imuDir = None       # Accelerometer - Magnetometer direction
encMarker = None    # Encoder - Magnetometer position
encDir = None       # Encoder - Magnetometer direction
visualMarker = None # VISO2 position
visualDir = None    # VISO2 direction
orbMarker = None    # ORB-SLAM2 position
orbDir = None       # ORB-SLAM2 direction

# Other display data
pwmText = None


class PosPlot:
    def __init__(self):
        # Datas of the different algorithms
        self.imuX = 0     # [m]
        self.imuY = 0     # [m]
        self.imuAngle = 0 # [rad]

        self.encX = 0     # [m]
        self.encY = 0     # [m]
        self.encAngle = 0 # [rad]

        self.visualX = 0
        self.visualY = 0
        self.visualAngle = 0 # [rad]
        self.visualAngleOffset = None

        self.orbX = 0
        self.orbY = 0
        self.orbAngle = 0 # [rad]
        self.orbAngleOffset = None

        self.pwm_freq = 0

        # Make subscriptions
        self.inercSub = rospy.Subscriber('/robopro/pose/inerc', Pose, self.inercCallback)
        self.encSub = rospy.Subscriber('/robopro/pose/encoder', Pose, self.encCallback)
        self.visualSub = rospy.Subscriber('/robopro/pose/viso2', PoseStamped, self.visualCallback)
        self.orbSub = rospy.Subscriber('/robopro/pose/orb_slam2', PoseStamped, self.orbCallback)
        self.pwm_freqSub = rospy.Subscriber('/robopro/status/pwm_freq', Int16, self.pwm_freqCallback)

    # Handles the encoder-magnetometer messages
    def encCallback(self, msg):
        self.encX = msg.position.x
        self.encY = msg.position.y
        self.encAngle = msg.orientation.z

    # Handles the accelerometer-magnetometer messages
    def inercCallback(self, msg):
        self.imuX = msg.position.x
        self.imuY = msg.position.y
        self.imuAngle = msg.orientation.z
        if self.visualAngleOffset is None:
            self.visualAngleOffset = msg.orientation.z

        if self.orbAngleOffset is None:
            self.orbAngleOffset = msg.orientation.z

    # Handles VISO2 messages
    def visualCallback(self, msg):
        self.visualX = msg.pose.position.x
        self.visualY = msg.pose.position.y
        # Convert from quaternion to single angle
        q = msg.pose.orientation
        len = math.sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)
        q.w = q.w/len
        q.x = q.x/len
        q.y = q.y/len
        q.z = q.z/len
        singularityTest = q.x*q.y + q.z*q.w
        if singularityTest > 0.499999: #north-pole singularity
            self.visualAngle = 2 * math.atan2(q.x,q.w)
        elif singularityTest < -0.499999:
            self.visualAngle = -2 * math.atan2(q.x,q.w)
        else:
            self.visualAngle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # if self.visualAngleOffset is None:
        #     self.visualAngle = self.visualAngle + self.visualAngleOffset

    # Handles ORB-SLAM2 messages
    def orbCallback(self, msg):
        self.orbX = msg.pose.position.x
        self.orbY = msg.pose.position.y
        q = msg.pose.orientation
        # Convert from quaternion to single angle
        len = math.sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)
        q.w = q.w/len
        q.x = q.x/len
        q.y = q.y/len
        q.z = q.z/len
        singularityTest = q.x*q.y + q.z*q.w
        if singularityTest > 0.499999: #north-pole singularity
            self.orbAngle = 2 * math.atan2(q.x, q.w)
        elif singularityTest < -0.499999:
            self.orbAngle = -2 * math.atan2(q.x, q.w)
        else:
            self.orbAngle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # if self.orbAngleOffset is None:
        #     self.orbAngle = self.orbAngle + self.orbAngleOffset

    # Handles PWM messages
    def pwm_freqCallback(self, msg):
        self.pwm_freq = msg.data

    # Performs drawing
    def animate(self):
        # Scale to pixel size
        imuHead = float(self.imuAngle) * math.pi / 180
        Ximu = float(self.imuX) * PIXEL_PER_METER
        Yimu = float(self.imuY) * PIXEL_PER_METER
        encHead = float(self.encAngle) * math.pi / 180
        Xenc = float(self.encX) * PIXEL_PER_METER
        Yenc = float(self.encY) * PIXEL_PER_METER
        visualHead = float(self.visualAngle) * math.pi / 180
        Xvisual = float(self.visualX) * PIXEL_PER_METER
        Yvisual = float(self.visualY) * PIXEL_PER_METER
        orbHead = float(self.orbAngle)
        Xorb = float(self.orbX) * 1 * PIXEL_PER_METER
        Yorb = float(self.orbY) * 1 * PIXEL_PER_METER

        # Translate origo to the center of the window and calculate pixel coords
        imuDrawX = WIDTH / 2 - Yimu
        imuDrawY = HEIGHT / 2 - Ximu
        imuDrawDirX = -math.sin(imuHead)
        imuDrawDirY = math.cos(imuHead)
        encDrawX = WIDTH / 2 - Yenc
        encDrawY = HEIGHT / 2 - Xenc
        encDrawDirX = -math.sin(encHead)
        encDrawDirY = math.cos(encHead)
        visualDrawX = WIDTH / 2 - Yvisual
        visualDrawY = HEIGHT / 2 - Xvisual
        visualDrawDirX = -math.sin(visualHead)
        visualDrawDirY = math.cos(visualHead)
        orbDrawX = WIDTH / 2 - Yorb
        orbDrawY = HEIGHT / 2 - Xorb
        orbDrawDirX = -math.sin(orbHead)
        orbDrawDirY = math.cos(orbHead)

        # Draw tracks
        C.create_oval(imuDrawX - SMALL_RADIUS, imuDrawY - SMALL_RADIUS, imuDrawX + SMALL_RADIUS, imuDrawY + SMALL_RADIUS, fill="green", outline="green")
        C.create_oval(encDrawX - SMALL_RADIUS, encDrawY - SMALL_RADIUS, encDrawX + SMALL_RADIUS, encDrawY + SMALL_RADIUS, fill="red", outline="red")
        C.create_oval(visualDrawX - SMALL_RADIUS, visualDrawY - SMALL_RADIUS, visualDrawX + SMALL_RADIUS, visualDrawY + SMALL_RADIUS, fill="purple", outline="purple")
        C.create_oval(orbDrawX - SMALL_RADIUS, orbDrawY - SMALL_RADIUS, orbDrawX + SMALL_RADIUS, orbDrawY + SMALL_RADIUS, fill="purple", outline="orange")

        # Move markers to the right position
        C.coords(imuMarker, imuDrawX - RADIUS, imuDrawY - RADIUS, imuDrawX + RADIUS, imuDrawY + RADIUS)
        C.coords(imuDir, imuDrawX, imuDrawY, imuDrawX + imuDrawDirX * 2 * RADIUS, imuDrawY - imuDrawDirY * 2 * RADIUS)

        C.coords(encMarker, encDrawX - RADIUS, encDrawY - RADIUS, encDrawX + RADIUS, encDrawY + RADIUS)
        C.coords(encDir, encDrawX, encDrawY, encDrawX + encDrawDirX * 2 * RADIUS, encDrawY - encDrawDirY * 2 * RADIUS)

        C.coords(visualMarker, visualDrawX - RADIUS, visualDrawY - RADIUS, visualDrawX + RADIUS, visualDrawY + RADIUS)
        C.coords(visualDir, visualDrawX, visualDrawY, visualDrawX + visualDrawDirX * 2 * RADIUS,
                 visualDrawY - visualDrawDirY * 2 * RADIUS)

        C.coords(orbMarker, orbDrawX - RADIUS, orbDrawY - RADIUS, orbDrawX + RADIUS, orbDrawY + RADIUS)
        C.coords(orbDir, orbDrawX, orbDrawY, orbDrawX - orbDrawDirX * 2 * RADIUS,
                 orbDrawY - orbDrawDirY * 2 * RADIUS)

        # Update displayed texts
        C.itemconfig(pwmText, text="PWM freq: {0}".format(self.pwm_freq))

        C.pack()
        C.after(20, self.animate)


if __name__ == "__main__":
    global imuMarker
    global imuDir
    global encMarker
    global encDir
    global visualMarker
    global visualDir
    global orbMarker
    global orbDir
    global pwmText

    # Draw squares
    for i in range(int(WIDTH / PIXEL_PER_DM)):                                        
        if i%10 == 0:                                                                 
            C.create_line(0, i * PIXEL_PER_DM, HEIGHT, i * PIXEL_PER_DM, fill="gray60")
        elif i%10 == 5:                                                               
            C.create_line(0, i * PIXEL_PER_DM, HEIGHT, i * PIXEL_PER_DM, fill="gray45")
        else:                                                                         
            C.create_line(0, i * PIXEL_PER_DM, HEIGHT, i * PIXEL_PER_DM, fill="gray30")

    for i in range(int(HEIGHT / PIXEL_PER_DM)):                                       
        if i%10 == 0:                                                                 
            C.create_line(i * PIXEL_PER_DM, 0, i * PIXEL_PER_DM, WIDTH, fill="gray60")
        elif i%10 == 5:                                                               
            C.create_line(i * PIXEL_PER_DM, 0, i * PIXEL_PER_DM, WIDTH, fill="gray45")
        else:                                                                         
            C.create_line(i * PIXEL_PER_DM, 0, i * PIXEL_PER_DM, WIDTH, fill="gray30")

    # Draw X-Y axis
    C.create_line(WIDTH/2, 0, WIDTH/2, HEIGHT, fill="white", width=1)
    C.create_line(0, HEIGHT/2, WIDTH, HEIGHT/2, fill="white", width=1)

    # Draw test route
    # for i in range(4):
    #     angle = i*math.pi/2 + math.pi/4
    #     C.create_line(WIDTH/2, HEIGHT/2, WIDTH/2 + math.cos(angle)*PIXEL_PER_METER, HEIGHT/2 + math.sin(angle)*PIXEL_PER_METER, fill="yellow", width=2)
    #     C.create_line( WIDTH/2 + math.cos(angle)*PIXEL_PER_METER, HEIGHT/2 + math.sin(angle)*PIXEL_PER_METER,WIDTH/2, HEIGHT/2 + 2 * math.sin(angle)*PIXEL_PER_METER, fill="yellow", width=2)

    # Initialize markers
    imuMarker = C.create_oval(WIDTH / 2 - RADIUS, HEIGHT / 2 - RADIUS, WIDTH / 2 + RADIUS, HEIGHT / 2 + RADIUS,
                              fill="green")
    imuDir = C.create_line(WIDTH / 2, HEIGHT / 2, WIDTH / 2 + 0 * 2 * RADIUS, HEIGHT / 2 - 1 * 2 * RADIUS, fill="green",
                           width=3)

    encMarker = C.create_oval(WIDTH / 2 - RADIUS, HEIGHT / 2 - RADIUS, WIDTH / 2 + RADIUS, HEIGHT / 2 + RADIUS,
                              fill="red")
    encDir = C.create_line(WIDTH / 2, HEIGHT / 2, WIDTH / 2 + 0 * 2 * RADIUS, HEIGHT / 2 - 1 * 2 * RADIUS, fill="red",
                           width=3)

    visualMarker = C.create_oval(WIDTH / 2 - RADIUS, HEIGHT / 2 - RADIUS, WIDTH / 2 + RADIUS, HEIGHT / 2 + RADIUS,
                                 fill="purple")
    visualDir = C.create_line(WIDTH / 2, HEIGHT / 2, WIDTH / 2 + 0 * 2 * RADIUS, HEIGHT / 2 - 1 * 2 * RADIUS,
                              fill="purple", width=3)

    orbMarker = C.create_oval(WIDTH / 2 - RADIUS, HEIGHT / 2 - RADIUS, WIDTH / 2 + RADIUS, HEIGHT / 2 + RADIUS,
                              fill="orange")
    orbDir = C.create_line(WIDTH / 2, HEIGHT / 2, WIDTH / 2 + 0 * 2 * RADIUS, HEIGHT / 2 - 1 * 2 * RADIUS,
                           fill="orange", width=3)

    pwmText = C.create_text(3, 3, text="PWM frequency:", fill="white", anchor="nw")

                          
    rospy.init_node("robopro_pos_plot")
    pp = PosPlot()

    C.pack()
    pp.animate()
    top.mainloop()
