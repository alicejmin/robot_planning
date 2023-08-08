# import ctypes

# clibrary = ctypes.CDLL("/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/RoadmapGenerator.cpp")

# clibrary.func()

#!/usr/bin/env python3

from std_msgs.msg import String
import rospy

class UserInputManager:
    def __init__(self):
        self.is_replanning = False
        self.drone_info = None

    def handle_user_input(self, user_input):
        if not self.is_replanning:
            if user_input == "y":
                drone_position = input("Enter the drone position (x y z): ")
                self.drone_info = (drone_position)
                self.is_replanning = True
        else:
            if user_input == "n":
                self.is_replanning = False
            elif user_input == "y":
                drone_position = input("Enter the drone position (x y z): ")
                self.drone_info = (drone_position)

def user_input_callback(data):
    user_input_manager.handle_user_input(data.data)

def user_input_node():
    rospy.init_node('user_input_node', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.Subscriber('chatter', String, user_input_callback)

    rate = rospy.Rate(1)  # Publish rate (1 Hz)
    is_replanning = False

    while not rospy.is_shutdown():
        if not is_replanning:
            user_input = input("Do you want to replan? Enter [y/n]: ")
            if user_input == "y":
                is_replanning = True
            pub.publish(user_input)
        else:
            rospy.sleep(5)  # Wait for 1 second during replanning
            is_replanning = False

if __name__ == '__main__':
    try:
        user_input_manager = UserInputManager()
        user_input_node()
    except rospy.ROSInterruptException:
        pass