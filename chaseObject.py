#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Pose2D

raspicam_width = 410
raspicam_height = 308
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
ang_spd_max = 0.5 # Kp for angle
lin_spd_max = 1  # Kp for distance

#get data when received from getObjectRange
def callback(data):
    # Get orientation and distance of the detected ball
    lidar_dist = data.x
    desired_angle = data.theta
    # print(lidar_dist, desired_angle)

    angular_spd = desired_angle/31.1  # Normalize the angle
    move_cmd = Twist()
    move_cmd.angular.z = ang_spd_max*angular_spd
    # move_cmd.angular.z = 0.0

    # Range of lidar is 0.1 - 3.5
    move_cmd.linear.x = lin_spd_max*(lidar_dist - 0.5)/3.4

    # when in dead zone
    if np.abs(angular_spd) < .05:
        move_cmd.angular.z = 0

    if desired_angle == 0:
        move_cmd.angular.z = 0

    # No reading / out of range values from lidar, don't move linearly
    if lidar_dist == 0:
        move_cmd.linear.x = 0


    print(move_cmd)
    pub.publish(move_cmd)
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_wheels', anonymous=True)

    rospy.Subscriber("ChaseObject",Pose2D,callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
