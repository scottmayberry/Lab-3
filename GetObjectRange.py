#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose2D
from time import time

lQ = []  # to store lidar range data
dQ = []  # to store camera data

max_queue_size = 3
max_millis_between_reads = 1000

prev_ballAngle = 0

#rospy publish node
pub = rospy.Publisher('/ChaseObject', Pose2D, queue_size=10)

#rospy init node
rospy.init_node('Convert_Cam_Lidar_to_Range', anonymous=True)

def getCurrentMillis():
    # gets current time in milliseconds. Stored alomg with data received from the 2 sensors
    return int(round(time() * 1000))

def detectObjectCameraCallback(data):
    global dQ
    # print('Camera callback')
    dQ.append([data, getCurrentMillis()])
    # print("Camera Callback: {}".format(dQ))

def lidarScanCallback(data):
    global lQ
    # print('lidar callback')
    lQ.append([data, getCurrentMillis()])
    # print("Lidar callback: {}".format(lQ))


def removeExcessMeasurements():
    global lQ, dQ, prev_ballAngle
    while(len(lQ) > max_queue_size):
        lQ.pop(0)
    while(len(dQ) > max_queue_size):
        ba = dQ.pop(0)[0]
        prev_ballAngle = int(getBallAngle(ba))


    return lQ,dQ

def removeOldTimeData():
    global max_millis_between_reads, lQ, dQ
    time_x = getCurrentMillis()

    # print(lQ, dQ)
    if(len(lQ) != 0):
        print("time_diff lQ: {}".format(time_x - lQ[0][1]))
        lQ = [x for x in lQ if (time_x - x[1]) > max_millis_between_reads]

    if(len(dQ) != 0):
        print("time_diff dQ: {}".format(time_x - dQ[0][1]))
        dQ = [x for x in dQ if (time_x - x[1]) > max_millis_between_reads]

    return lQ, dQ

def getBallAngle(ballPosition):
    ballAngle = (410 - ballPosition.y)*(0.1517073)-31.1
    if ballPosition.y == -1 : return 0
    return ballAngle

# def getLidarAngle(ballAngle, laserData):
#     return laserData.ranges[ballAngle]
    
def getLidarAngle(ballAngle, laserData):
    # Use median of values of ranges +/- 5 from the desired angle for lidar dist
    MIN, MAX = -5, 5
    distances = []
    for d_ang in range(MIN, MAX+1):
        distances.append(laserData.ranges[ballAngle+d_ang])
    return np.median(distances)

def getDistanceAndOrientation():
    global lQ, dQ, prev_ballAngle
    if(len(lQ) == 0 or len(dQ) == 0):
        # print("Empty queues")
        lidarValue = 0
        ballAngle = 0

    else:
        laserData = lQ.pop(0)[0]
        ballPosition = dQ.pop(0)[0]
        ballAngle = int(getBallAngle(ballPosition))
        
        print(ballAngle)
        lidarValue = getLidarAngle(ballAngle, laserData)
        

    if(lidarValue < 0.1) or (lidarValue > 3.5):
        # print("lidar is out of range")
        lidarValue = 0
        ballAngle = prev_ballAngle
    
    print("ball angle: {}".format(ballAngle))
    print("lidar value: {}".format(lidarValue))

    pose2d = Pose2D()
    pose2d.x = lidarValue
    pose2d.theta = ballAngle
    pub.publish(pose2d)

    return lQ, dQ


def listener():
    global lQ, dQ

    rospy.Subscriber("/get_object_range",Point,detectObjectCameraCallback,queue_size=max_queue_size)
    rospy.Subscriber("/scan",LaserScan,lidarScanCallback,queue_size=max_queue_size)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    # print("I'm spinnning")
    while True:
        removeExcessMeasurements()
        # removeOldTimeData()
        if len(lQ) > 0:
            print("While loop before : {}, {}".format(len(lQ), len(dQ)))
            getDistanceAndOrientation()
        # print("While loop after: {}, {}".format(len(lQ), len(dQ)))

        
        

if __name__ == '__main__':
    listener()
