#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point
from time import time

laserScanQueue = []
detectObjectQueue = []

max_queue_size = 3
max_millis_between_reads = 100

#rospy publish node
pub = rospy.Publisher('/ChaseObject', Point, queue_size=10)

#rospy init node
rospy.init_node('Convert_Cam_Lidar_to_Range', anonymous=True)

def getCurrentMillis():
    return int(round(time.time() * 1000))

def detectObjectCameraCallback(data):
    detectObjectQueue.append([data, getCurrentMillis])

def lidarScanCallback(data):
    laserScanQueue.append([data, getCurrentMillis])


def removeExcessMeasurements(lQ, dQ):
    while(len(lQ) > max_queue_size):
        lQ.pop(0)
    while(len(dQ) > max_queue_size):
        dQ.pop(0)
def removeOldTimeData(lQ, dQ):
    global max_millis_between_reads
    if(len(lQ) != 0):
        lQ = [x for x in lQ if x[1] - getCurrentMillis() > max_millis_between_reads]
    if(len(dQ) != 0):
        dQ = [x for x in lQ if x[1] - getCurrentMillis() > max_millis_between_reads]

def getDistanceAndOrientation(lQ, dQ):
    if(len(lQ) == 0 or len(dQ) == 0):
        return
    laserData = lQ.pop(0)[0]
    ballPosition = dQ.pop(0)[0]
    

def listener(lQ, dQ):
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('find_ball', anonymous=True)

    rospy.Subscriber("/get_object_range",Point,detectObjectCameraCallback,queue_size=max_queue_size)
    rospy.Subscriber("/scan",LaserScan,lidarScanCallback,queue_size=max_queue_size)

    # spin() simply keeps python from exiting until this node is stopped
    while True:
        removeExcessMeasurements(lQ,dQ)
        removeOldTimeData(lQ, dQ)
        getDistanceAndOrientation(lQ,dQ)
        
        

if __name__ == '__main__':
    listener(laserScanQueue, detectObjectQueue)
