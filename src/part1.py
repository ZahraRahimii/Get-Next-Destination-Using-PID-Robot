#! /usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
import time
from math import sqrt, pow

class PIDController():


    def __init__(self):
        
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.k_i = 0.05
        self.k_p = 0.1
        self.k_d = 0.1
        
        self.dt = 0.005
        # self.linear_x = 0.6
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []
        self.target_x = 10
        self.target_y = 0


    def euclidean_distance(self):
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        
        curr_x = odom_msg.pose.pose.position.x
        curr_y = odom_msg.pose.pose.position.y

        return sqrt(pow((self.target_x - curr_x), 2) + pow((self.target_y - curr_y), 2))
    
    def reach_point(self):
        
        d = self.euclidean_distance()    
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        # move_cmd.angular.z = 0
        move_cmd.linear.x = 0

        while not rospy.is_shutdown() and d > 0.5:
            
            self.cmd_vel.publish(move_cmd)

            err = d
            self.errs.append(err)
            sum_i_theta += err * self.dt
            
            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            print(f"P : {P} I : {I} D : {D}")
            # move_cmd.angular.z = P + I + D 
            prev_theta_error = err
            move_cmd.linear.x =  P + I + D 
            
            print(f"error : {err} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")
            
            d = self.euclidean_distance()

            self.r.sleep()
            
        move_cmd.linear.x = 0
        self.cmd_vel.publish(move_cmd)


    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)
            

if __name__ == '__main__':
    try:
        pidc = PIDController()
        pidc.reach_point()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

