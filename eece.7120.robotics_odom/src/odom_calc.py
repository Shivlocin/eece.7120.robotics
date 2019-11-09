#!/usr/bin/env python
from math import sin,cos,pi
import rospy
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt


class Odom_Calc:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_list = list()
        self.y_list = list()

    def ticks_cb(self, msg):
        l = msg.x
        r = msg.y
        delta_theta = 0.0
        # add your code to calculate odometry here
        delta_theta = (r - l) / 0.1
        delta_s = (r+l)/2
        deltacos = cos(self.theta + delta_theta/2)
        deltasin = sin(self.theta + delta_theta/2)
        delta_x = delta_s * deltacos
        delta_y = delta_s * deltasin

        # make sure to update self.x, self.y, self.theta
        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.theta = self.theta + delta_theta

        if self.theta > pi:
            self.theta -= 2*pi
        if self.theta < -pi:
            self.theta += 2*pi
        self.x_list.append(self.x)
        self.y_list.append(self.y)

        

if __name__ == '__main__':
    try:
        rospy.init_node('odom_calc', anonymous=True)
        oc = Odom_Calc()
        rospy.Subscriber("wheel_tick", Vector3, oc.ticks_cb)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            plt.plot(oc.x_list, oc.y_list,'ro-',)
            plt.axis([0,1.5,-1,1])
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Vehicle Odometry')
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
