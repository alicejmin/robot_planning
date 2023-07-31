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
import matplotlib
matplotlib.use('Agg')

# Environment constants
Z = 1.0
TAKEOFF_DURATION = 2.5
TARGET_HEIGHT = 0.02
GOTO_DURATION = 3.0
LAND_DURATION = 3.0
planned_path_data = ""
planned_path_data = []


swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

def planned_path_callback(msg):
    
    
    """ MISSION """
    allcfs.takeoff(0.02, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1)  

    # Extract coordinates from the received message
    points = msg.data.strip().split(';')
    for point in points:
        if not point:
            continue
        x, y, z = point.split(',')
        x, y, z = float(x), float(y), float(z)
        planned_path_data.append((x, y, z))

    rospy.loginfo("Received planned_path: %s", planned_path_data)
    
    for wp in planned_path_data: 
        cf = allcfs.crazyflies[0]
        cf.goTo(wp, 0.0, GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + 0.05)
    timeHelper.sleep(GOTO_DURATION + 0.05)

    # Land the drones
    allcfs.land(TARGET_HEIGHT, LAND_DURATION)
    timeHelper.sleep(LAND_DURATION + 1)

def broadcast_pos(rate, cf):
    # rospy.init_node('robot_position')
    #Publish the current position at the specified rate 
    rate = rospy.Rate(rate)
    pub = rospy.Publisher('robot_pos', Point, queue_size=10)
    while not rospy.is_shutdown():
        pos = Point()
        pos.x, pos.y, pos.z = cf.position()
        pub.publish(pos)
        print(pos)
        rate.sleep()

def main():
    # ROS node initialization
    rospy.init_node('subscriber', anonymous=True)

    # Send out robot locations
    from threading import Thread
    Thread(target=broadcast_pos, kwargs={"rate": 1, "cf": allcfs.crazyflies[0]}).start()

    # Subscriber for the "planned_path" topic
    rospy.Subscriber("planned_path", String, planned_path_callback)
    
    # Keep the node running until it's shut down
    rospy.sleep(60)


if __name__ == "__main__":
    main()
