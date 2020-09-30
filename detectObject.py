#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

#threshold values for HSV filter for pink volleyball
min_neon_pink = np.array([163, 21, 90])
max_neon_pink = np.array([177, 180, 255])

#rospy publish node
pub = rospy.Publisher('/get_object_range', Point, queue_size=10)

#rospy init node
rospy.init_node('detect_object', anonymous=True)

#cv bridge used for converting pi camera data to numpy
br = CvBridge()
#added this test line

def findCenterPixel(img_sen, min_mask_value, max_mask_value, maskIteration=1):
    # convert live stream to HSV
    hsv_stream = cv2.cvtColor(img_sen, cv2.COLOR_BGR2HSV)
    # make a mask of the filter ranging from min value to max value
    mask1 = cv2.inRange(hsv_stream,
                        min_mask_value, max_mask_value)
    #print(mask1)
    mask = np.array(mask1!=0)
    xs,ys = np.where(mask)
    #print(xs.shape)
    if xs.shape[0] < 500: return -1,-1
    x = int(xs.median())
    y = int(ys.median())
    print(x,y)
    return x,y

def callback(data):
    global min_neon_pink, max_neon_pink, pub
    img_sen = br.compressed_imgmsg_to_cv2(data)
    x,y = findCenterPixel(img_sen, min_neon_pink, max_neon_pink)
    hough_circle_msg = Point(x, y, 0)
    pub.publish(hough_circle_msg)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('find_ball', anonymous=True)

    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,callback,queue_size=3)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
