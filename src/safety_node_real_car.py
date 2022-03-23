#!/usr/bin/env python
import rospy
from cmath import isinf, isnan, cos
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3, _TwistWithCovariance

global vx, vy, lidar_info, brake_bool_result, Ackermann_drive_result, TTC_threshold, min_TTC
Ackermann_drive_result = AckermannDriveStamped()

pub1 = rospy.Publisher('/brake_bool', Bool, queue_size = 10)
pub2 = rospy.Publisher('/brake', AckermannDriveStamped, queue_size = 10)
ROS_INFO("Testing for clear car")

# TODO: import ROS msg types and libraries

class Safety(object):
   
    def __init__(self):
       
        sub1 = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 10)
        sub2 = rospy.Subscriber('vesc/odom', Odometry, self.odom_callback, queue_size = 10)


    def odom_callback(self, odom_msg):

    	self.odom_msg = odom_msg
    	self.vx = odom_msg.twist.twist.linear.x
    	self.vy = odom_msg.twist.twist.linear.y
       

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        self.lidar_info = scan_msg
        # self.lidar_angle_min = scan_msg.angle_min     
        # self.lidar_angle_increment = scan_msg.angle_increment
        # self.lidar_ranges = scan_msg.ranges
        TTC_threshold = 0.6
        min_TTC = 100

        for x in range(len(self.lidar_ranges)):
	    
            if (not (math.isinf(self.scan_msg.ranges[x])) and not (math.isnan(self.scan_msg.ranges[x]))):
                # self.distance = self.lidar_ranges[x]
                # self.angle = self.lidar_angle_min + self.lidar_angle_increment * x
                # self.distance_rate = math.cos(self.angle)*self.vx
                # self.distance_rate = max(self.distance_rate, 0)

                # if(self.distance_rate != 0.0 and self.distance/self.distance_rate < min_TTC):
                #     min_TTC = self.distance/self.distance_rate
                #     print("min_ttc: ", min_TTC)

                if(min_TTC < TTC_threshold):
                    brake_bool_result = bool(True)
                    pub1.publish(brake_bool_result)
                    Ackermann_drive_result.drive.speed = 1.0
                    pub2.publish(Ackermann_drive_result)        	
                    print("brake applied")
                else:
                    brake_bool_result = bool(False)
                    pub1.publish(brake_bool_result)
                    #print("brake failed")

        # TODO: publish brake message and publish controller bool


def main():
    rospy.init_node('safety_node', anonymous=True)

    while not rospy.is_shutdown():
	    sn = Safety()
	    rospy.spin()
	    

if __name__ == '__main__':
    main()
