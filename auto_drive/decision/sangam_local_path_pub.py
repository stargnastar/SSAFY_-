#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("global_path", Path, self.global_path_callback)

        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1) 

        self.is_odom = False
        self.is_path = False
        self.local_path_size = 100  # Can be adjusted based on your specific needs

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y


                min_dis = float('inf')
                current_waypoint = -1
                for i, pose in enumerate(self.global_path_msg.poses):
                    distance = sqrt(pow((pose.pose.position.x - x), 2) + pow((pose.pose.position.y - y), 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:current_waypoint+self.local_path_size]
                    else:
                        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]

                if local_path_msg.poses:
                    rospy.loginfo("Local path message is not empty.")
                
                print(x,y)
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg        

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
