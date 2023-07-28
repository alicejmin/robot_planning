#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
import sys
import os
sys_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(sys_path)
from pycrazyswarm import *

# Environment constants
Z = 1.0
TAKEOFF_DURATION = 2.5
TARGET_HEIGHT = 0.02
GOTO_DURATION = 3.0
LAND_DURATION = 3.0
planned_path_data = ""

def planned_path_callback(msg):
   
    global planned_path_data
    planned_path_data = []
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    """ MISSION """
    allcfs.takeoff(Z, TAKEOFF_DURATION)
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

    # Land the drones
    allcfs.land(TARGET_HEIGHT, LAND_DURATION)
    timeHelper.sleep(LAND_DURATION + 1)


def main():
    # ROS node initialization
    rospy.init_node('subscriber', anonymous=True)

    # Subscriber for the "planned_path" topic
    rospy.Subscriber("planned_path", String, planned_path_callback)

    # Keep the node running until it's shut down
    rospy.sleep(60)

if __name__ == "__main__":
    main()