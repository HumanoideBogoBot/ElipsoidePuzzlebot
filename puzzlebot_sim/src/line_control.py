#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.stats import norm

class Movimiento():
    
    def __init__(self):
        rospy.init_node("movimiento")
        self.tiempo_final = 15
        self.tiempo_actual = 0
        self.rate = rospy.Rate(100)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)
        rospy.Subscriber("/rviz/wl", Float32, self.wl_callback)
        rospy.Subscriber("/rviz/wr", Float32, self.wr_callback)
        rospy.Subscriber("/pose_sim",PoseStamped,self.true_odometry_callback)
        rospy.wait_for_service('/reset')
        self.reset = rospy.ServiceProxy('/reset', Empty)
        self.linear_vel = 0.0
        self.angular_vel = 0.1
        self.wr = 0
        self.wl = 0
        self.msg_vel = Twist()
        self.true_odometry = [0.0,0.0,0.0]
        self.posiciones_reales = []
    
    def wr_callback(self,data):
        self.wr = data.data
    def wl_callback(self,data):
        self.wl = data.data
    
    def true_odometry_callback(self,data):
        header = data.header.stamp
        self.true_odometry = [euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])[2],
                                data.pose.position.x,
                                data.pose.position.y]

    def main(self):
        t0 = rospy.get_rostime().to_sec()
        repeticiones = 20
        for i in range(repeticiones):
            while self.tiempo_actual <= self.tiempo_final:
                self.msg_vel.linear.x = self.linear_vel
                self.msg_vel.angular.z = self.angular_vel
                self.cmd_vel_pub.publish(self.msg_vel)
                t1 = rospy.get_rostime().to_sec()
                delta_t = t1 - t0
                if delta_t < 0.05:
                    self.tiempo_actual += delta_t
                self.rate.sleep()
                t0 = t1
            #print("Tiempo :",self.tiempo_actual,"Angulo: ",self.angulo_real,"Pisicion X: ",self.x_real,"Posicion Y: ",self.y_real)
            print("Tiempo :",self.tiempo_actual,"Angulo real: ",self.true_odometry[0],"Pisicion X real: ",self.true_odometry[1],"Posicion Y real: ",self.true_odometry[2])
            self.msg_vel.linear.x = 0
            self.msg_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.msg_vel)
            #self.posiciones_aprox.append((self.angulo_real,self.x_real,self.y_real))
            self.posiciones_reales.append((self.true_odometry[0],self.true_odometry[1],self.true_odometry[2]))
            self.tiempo_actual = 0
            self.x_real = 0
            self.y_real = 0
            self.angulo_real = 0
            self.reset()
        x_real = []
        y_real = []
        q_real = []
        for real in self.posiciones_reales:
            q,x,y = real
            x_real.append(x)
            y_real.append(y)
            q_real.append(q)
        print("X:")
        print("Mean real: ",np.mean(x_real))
        print("Std real: ",np.std(x_real))
        print("Var real: ",np.var(x_real))
        print("Y:")
        print("Mean real: ",np.mean(y_real))
        print("Std real: ",np.std(y_real))
        print("Var real: ",np.var(y_real))
        print("Q:")
        print("Mean real: ",np.mean(q_real))
        print("Std real: ",np.std(q_real))
        print("Var real: ",np.var(q_real))
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(y_real, x_real, s=0.5)
        fig.subplots_adjust(hspace=0.25)
        plt.title("Puntos finales en X Y")
        plt.xlabel("Posicion en Y")
        plt.ylabel("Posicion en X")
        plt.savefig('puntos.png')
        plt.show()
        fig, ax = plt.subplots(figsize=(6, 6))
        plt.title("Histograma se posicion en x")
        plt.xlabel("Posicion en x")
        plt.ylabel("Frecuencia")
        plt.hist(x_real)
        x_axis = np.linspace(np.min(x_real),np.max(x_real))
        mean = np.mean(x_real)
        std = np.std(x_real,ddof = 1)
        plt.plot(x_axis, norm.pdf(x_axis, mean, std),color = "r")
        plt.savefig('histo_puntos.png')
        plt.show()
        fig, ax = plt.subplots(figsize=(6, 6))
        plt.title("Histograma de angulo de giro")
        plt.xlabel("Angulo de giro final")
        plt.ylabel("Frecuencia")
        plt.hist(q_real)
        q_axis = np.linspace(np.min(q_real),np.max(q_real))
        mean = np.mean(q_real)
        std = np.std(q_real,ddof = 1)
        plt.plot(q_axis, norm.pdf(q_axis, mean, std),color = "r")
        plt.savefig('histo_angle.png')
        plt.show()
        
if __name__ == "__main__":
    robot = Movimiento()
    robot.main()
