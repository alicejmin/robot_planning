#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import sys
import os
sys_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(sys_path)
from pycrazyswarm import *
import rob_operate

cf = rob_operate.cf

swarm = Crazyswarm()
# timeHelper = swarm.timeHelper
cf = swarm.allcfs.crazyflies[0]

def main():
    rospy.init_node('robot_pos_node', anonymous=True)
    pub = rospy.Publisher('/robot_pos', Point, queue_size=1, tcp_nodelay=False)
    
    loop_hz = 60
    rate = rospy.Rate(loop_hz)
    while not rospy.is_shutdown():
        pos = Point()
        pos.x, pos.y, pos.z = cf.position()
        pub.publish(pos)
        print(pos)
        rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass