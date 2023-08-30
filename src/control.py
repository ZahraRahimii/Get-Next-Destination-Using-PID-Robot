#!/usr/bin/python3

import rospy
from step1.srv import GetNextDstSrv
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from math import radians, pi, atan2, sqrt, pow
import numpy as np
import matplotlib.pyplot as plt
import math


class Control:

    def __init__(self) -> None:
        
        rospy.init_node("controller_node" , anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist , queue_size=10)
        
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.curr_x = 0.0; self.next_x = self.curr_x
        self.curr_y = 0.0; self.next_y = self.curr_y

        # gains for angular PID controller
        # self.kp_a = 0.6
        self.kp_a = 0.4
        self.ki_a = 0.001
        # self.kd_a = 1.2
        self.kd_a = 0.2

        # gains for linear PID controller
        # self.kp_l = 0.1
        self.kp_l = 0.1
        self.ki_l = 0.001
        # self.kd_l = 0.2
        self.kd_l = 0.005

        self.dt = 0.05
        self.epsilon = 0.2
        self.errors = []
        self.orientaion = 0
        self.goal_orientaion = 0


        
    # heading of the robot 
    def get_heading(self):
        
        'waiting for the most recent message from topic /odom'
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y

        self.curr_x = self.x_pose
        self.curr_y = self.y_pose
        'convert quaternion to odom'
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw

    def updata_pos(self):
        msg = rospy.wait_for_message("/odom" , Odometry)

        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y

        self.curr_x = x_pose
        self.curr_y = y_pose

    # def run(self):
        
    #     for i in range(5):
            
    #         r = rospy.Rate(self.linear_speed * 10)

    #         print(f'\nIteration {i + 1}:\n')
        
    #         self.next_x, self.next_y = get_nxt_dst(self.curr_x, self.curr_y)
            
    #         print(f"\n\nnext x \t{self.next_x}")
    #         print(f"\n\nnext y \t{self.next_y}\n\n")
            
    #         inc_x = self.next_x - self.curr_x
    #         inc_y = self.next_y - self.curr_y
            
    #         angle_to_dst = atan2(inc_y, inc_x)

    #         if angle_to_dst > pi:
    #             angle_to_dst = (angle_to_dst + 2 * pi)
    #         elif angle_to_dst < -pi:
    #             angle_to_dst = (angle_to_dst - 2 * pi)

    #         yaw = self.get_heading()
    #         remaining = yaw - angle_to_dst

    #         'Roatate to the target angle'
    #         if abs(remaining) > 0.01:  
    #             epsilon = 0.01
    #             self.rotation(remaining, epsilon)
    #         self.make_twist(0, 0)
    #         r.sleep()

    #         'Move forward to the target point with telorance of 2'
    #         distance_to_dst = two_points_distance(self.curr_x, self.curr_y, self.next_x, self.next_y)
    #         if distance_to_dst > 0.1:
    #             self.move_forward(distance_to_dst)
    #         self.updata_pos()
    #         self.make_twist(0, 0)
    #         r.sleep()

    def get_angular_error(self, next_x, next_y):
            
        inc_x = next_x - self.curr_x
        inc_y = next_y - self.curr_y
        self.goal_orientaion = atan2(inc_y, inc_x)

        # if self.goal_orientaion > pi:
        #     self.goal_orientaion = (self.goal_orientaion + 2 * pi)
        # elif self.goal_orientaion < -pi:
        #     self.goal_orientaion = (self.goal_orientaion - 2 * pi)

        # return self.orientaion - self.goal_orientaion
        if self.orientaion > 0:
            sign = -1 if (self.orientaion - math.pi < self.goal_orientaion < self.orientaion) else +1
        else:
            sign = +1 if (self.orientaion < self.goal_orientaion < self.orientaion + math.pi) else -1

        return sign * (math.pi - abs(abs(self.orientaion - self.goal_orientaion) - math.pi))


    def get_linear_error(self, next_x, next_y):

        return two_points_distance(self.curr_x, self.curr_y, next_x, next_y)
    
    def PID_controller(self):

            twist = Twist()
            sum_angular_error = 0
            sum_linear_error = 0
            prev_angular_error = 0
            prev_linear_error = 0
    
            for i in range(3):
                self.next_x = 9
                self.next_y = -8

                self.next_x, self.next_y = get_nxt_dst(self.curr_x, self.curr_y)
                distance = self.get_linear_error(self.next_x, self.next_y)
                
                while distance < 10:
                    self.next_x, self.next_y = get_nxt_dst(self.curr_x, self.curr_y)
                    distance = self.get_linear_error(self.next_x, self.next_y)
                
                print(f"\n\nnext x \t{self.next_x}")
                print(f"\n\nnext y \t{self.next_y}\n\n")
                distance = self.get_linear_error(self.next_x, self.next_y)

                while distance > 0.2:
                    print(f'distance: {distance}')
                    self.orientaion = self.get_heading()
                    self.updata_pos()
    
                    linear_error = self.get_linear_error(self.next_x, self.next_y)
                    angular_error = self.get_angular_error(self.next_x, self.next_y)
                    
                    self.errors.append(linear_error)
                    sum_linear_error += (linear_error * self.dt)
                    sum_angular_error += (angular_error * self.dt)

                    # calculate PID for linear speed
                    P = self.kp_l * linear_error
                    I = self.ki_l * sum_linear_error
                    D = self.kd_l * (linear_error - prev_linear_error)
                    twist.linear.x =  P + I + D

                    # calculate PID for angular speed
                    P = self.kp_a * angular_error
                    I = self.ki_a * sum_angular_error
                    D = self.kd_a * (angular_error - prev_angular_error) 
                    twist.angular.z =  P + I + D

                    prev_angular_error = angular_error
                    prev_linear_error = linear_error

                    distance = self.get_linear_error(self.next_x, self.next_y)

                    self.cmd_publisher.publish(twist)
                    rospy.sleep(self.dt)

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_publisher.publish(Twist())
        
        plt.plot(list(range(len(self.errors))),
                    self.errors, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.show()

def two_points_distance(x1, y1, x2, y2):
        return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2))

def get_nxt_dst(x, y):
    rospy.wait_for_service('get_next_dst')
    try:
        next_dst = rospy.ServiceProxy('get_next_dst', GetNextDstSrv)
        resp1 = next_dst(x, y)
        return (resp1.next_x, resp1.next_y)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
 
    controller = Control()
    controller.PID_controller()
 