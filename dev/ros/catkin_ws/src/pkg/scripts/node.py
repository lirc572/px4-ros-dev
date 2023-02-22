#! /usr/bin/env python

"""
Code is adapted from https://docs.px4.io/main/en/ros/mavros_offboard_python.html
"""

import rospy

from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

current_imu_data = Imu()
def imu_data_cb(msg):
    global current_imu_data
    current_imu_data = msg

current_local_position_pose = PoseStamped()
def local_position_pose_cb(msg):
    global current_local_position_pose
    current_local_position_pose = msg

current_local_position_velocity = TwistStamped()
def local_position_velocity_cb(msg):
    global current_local_position_velocity
    current_local_position_velocity = msg

if __name__ == "__main__":
    rospy.init_node("my_node")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    # subscribe to the topics that we need
    imu_data_sub = rospy.Subscriber("mavros/imu/data", Imu, callback = imu_data_cb)
    local_position_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_position_pose_cb)
    local_position_velocity_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, callback = local_position_velocity_cb)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        print('Connecting to PX4...')
        rate.sleep()
    print('Connected!')

    rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        print('Running...')
        rate.sleep()
