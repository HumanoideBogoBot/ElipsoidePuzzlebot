#!/usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Odometry():
    
    def __init__(self):
        rospy.init_node('Gazebo_sim_pos')
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)
        self.pos_pub = rospy.Publisher("/gazebo/odometry",Odometry,queue_size = 10)
        self.wl = 0
        self.wr = 0
        self.r = 0.05
        self.l = 0.188
        self.x = 0
        self.y = 0
        self.th = 0
        self.v = 0.2
        self.w = 0.1
        self.rate = rospy.Rate(100)
    
    def wl_callback(self,data):
        self.wl = data.data
    
    def wr_callback(self,data):
        self.wr = data.data

    def odometry_call(self,wr,wl,th,dt):
        v_w_matrix = np.array([[0.5, 0.5]
                              ,[1/self.l,-1/self.l]])
        delta_r_l = np.array([[dt*wr*self.r],
                               [dt*wl*self.r]])
        delta_d_th = np.dot(v_w_matrix,delta_r_l)
        print(delta_d_th)

        return (delta_d_th[0,0]*np.cos(th),delta_d_th[0,0]*np.sin(th),delta_d_th[1,0])
    
    def main(self):
        t0 = rospy.get_rostime().to_sec()
        while not rospy.is_shutdown():
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.v
            cmd_vel_msg.angular.z = self.w
            self.cmd_vel_pub.publish(cmd_vel_msg)
            t1 = rospy.get_rostime().to_sec()
            dt = t1 - t0
            if dt < 1:
                dx,dy,dth = self.odometry_call(self.wr,self.wl,self.th,dt)
                self.x += dx
                self.y += dy
                self.th += dth
            print(self.x,self.y,self.th)
            t0 = t1
            self.rate.sleep()

if __name__ == "__main__":
    robot = Odometry()
    robot.main()

    
    