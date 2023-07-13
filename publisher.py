#!/usr/bin/env python3

## Simple talker demo that published std_msgs/Strings robot messages
## to the 'chatter' topic
import json
import rospy
from std_msgs.msg import String

def talker(json_data):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    if not rospy.is_shutdown(): 
        movements = json_data['movements']
        for timestamp, robots in movements.items():
            for robot in robots:
                robot_id = robot['robot']
                position = robot['position']
                x = position['x']
                y = position['y']
                z = position['z']
                r_pos = f'Timestamp: {timestamp}, Robot {robot_id}: (x={x}, y={y}, z={z})'
                rospy.loginfo(r_pos)
                pub.publish(r_pos)
                rate.sleep()

# Read the JSON file
with open('../data/utra_sim.json') as json_file:
    json_data = json.load(json_file)
    talker(json_data)
                
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

import json

