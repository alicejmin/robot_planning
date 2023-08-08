#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import sys
import os
sys_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(sys_path)
from pycrazyswarm import *
from threading import Thread
import matplotlib
matplotlib.use('Agg')

# Environment constants
Z = 1.0
TAKEOFF_DURATION = 2.0
TARGET_HEIGHT = 0.02
GOTO_DURATION = 1.0
LAND_DURATION = 2.0
planned_path_data = None


swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cf = allcfs.crazyflies[0]

def planned_path_callback(msg):
    global planned_path_data
    """ MISSION """

    # Extract coordinates from the received message
    if msg.data == "goal":
        planned_path_data = []
    else:
        points = msg.data.strip().split(';')
        planned_path_data = []
        for point in points:
            if not point:
                continue
            x, y, z = point.split(',')
            x, y, z = float(x), float(y), float(z)
            planned_path_data.append((x, y, z))

    rospy.loginfo("Received planned_path: %s", planned_path_data)

def publish_rob_pos():
    pub = rospy.Publisher('/robot_pos', Point, queue_size=10)
    
    loop_hz = 60
    rate = rospy.Rate(loop_hz)
    while not rospy.is_shutdown():
        pos = Point()
        pos.x, pos.y, pos.z = cf.position()
        pub.publish(pos)
        # print(pos)
        rate.sleep()


def main():
    global planned_path_data
    # ROS node initialization
    #rospy.init_node('rob_node', anonymous=True)
    
    # Subscriber for the "planned_path" topic
    rospy.Subscriber("/planned_path", String, planned_path_callback, queue_size=10)

    t = Thread(target=publish_rob_pos)
    t.start()
    
    allcfs.takeoff(0.02, TAKEOFF_DURATION)
    # timeHelper.sleep(TAKEOFF_DURATION + 1)  

    loop_hz = 1
    rate = rospy.Rate(loop_hz)
    while not rospy.is_shutdown():
        if planned_path_data == None:
            continue
        if len(planned_path_data) != 0:
            wp = planned_path_data[0] 
            planned_path_data = planned_path_data[1:]
            print("here")
            cf.goTo(wp, 0.0, GOTO_DURATION)
            timeHelper.sleep(GOTO_DURATION - 0.5)

        if len(planned_path_data) == 0:
            allcfs.land(TARGET_HEIGHT, LAND_DURATION)
            timeHelper.sleep(LAND_DURATION + 1)
            break
        rate.sleep()
    
    rospy.spin()


if __name__ == "__main__":
    main()