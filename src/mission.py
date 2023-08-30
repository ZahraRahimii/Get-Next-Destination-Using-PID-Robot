#!/usr/bin/python3

from __future__ import print_function

from step1.srv import GetNextDstSrv, GetNextDstSrvResponse
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import rospy
import random
from math import sqrt, pow

class Mission:
    def __init__(self) -> None:
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
        get_next_dst_server()
        
    def odom_callback(self, msg: Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

def two_points_distance(x1, y1, x2, y2):
        return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2))

def calculate_next(a):
    
    # karan_bala = min(a, 20)
    # karan_paeen = max(-20, a)
    random_num = random.uniform(-20, 20)
    # random_num = random.uniform(-20, a)

    
    return random_num

def handle_get_nxt_dst(req):
    next_x = calculate_next(req.current_x)
    next_y = calculate_next(req.current_y)

    print("Returning x = [%s], y = [%s] so that next_x = [%s] and next_y = [%s]"%(req.current_x, req.current_y, next_x, next_y))
    resp = GetNextDstSrvResponse()
    resp.next_x = next_x
    resp.next_y = next_y
    return resp

def get_next_dst_server():
    rospy.init_node('mission_node')
    s = rospy.Service('get_next_dst', GetNextDstSrv, handle_get_nxt_dst)
    print("Ready to get next destination.")

if __name__ == "__main__":
    
    mission = Mission()

    rospy.spin()

