#!/usr/bin/env python3
import time
import rospy
import sensor_msgs.msg
from std_msgs.msg import *
from hunter_msgs.msg import *
import psutil
import nvidia_smi
import socket
import rosnode



class usage:
    pubScvStatus = rospy.Publisher('/pubScvStatus', Float64MultiArray, queue_size=10)

    def __init__(self):
        self.hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, self.callback)
        self.scvMsgArr = Float64MultiArray()

        self.base_state = 0
        self.battery_voltage = 0
        self.control_mode = 0
        self.fault_code = 0
        self.linear_velocity = 0
        self.park_mode = 0
        self.steering_angle = 0

        self.driver_states_front = []
        self.driver_states_rareLeft = []
        self.driver_states_rareRight = []

        self.motor_states_front = []
        self.motor_states_rareLeft = []
        self.motor_states_rareRight = []

        self.motorStates = [self.motor_states_front, self.motor_states_rareLeft, self.motor_states_rareRight]
        self.driverStates = [self.driver_states_front, self.driver_states_rareLeft, self.motor_states_rareRight]


    def callback(self, data):
        #
        self.base_state = data.base_state
        self.battery_voltage = data.battery_voltage
        self.control_mode = data.control_mode
        self.fault_code = data.fault_code
        self.linear_velocity = data.linear_velocity
        self.park_mode = data.park_mode
        self.steering_angle = data.steering_angle

        self.driver_states_front = [data.driver_states[0].current, data.driver_states[0].rpm, data.driver_states[0].temperature, data.driver_states[0].motor_pose]
        self.driver_states_rareLeft = [data.driver_states[1].current, data.driver_states[1].rpm, data.driver_states[1].temperature, data.driver_states[1].motor_pose]
        self.driver_states_rareRight = [data.driver_states[2].current, data.driver_states[2].rpm, data.driver_states[2].temperature, data.driver_states[2].motor_pose]

        self.motor_states_front = [data.motor_states[0].driver_voltage, data.motor_states[0].driver_temperature, data.motor_states[0].driver_state]
        self.motor_states_rareLeft = [data.motor_states[1].driver_voltage, data.motor_states[1].driver_temperature, data.motor_states[1].driver_state]
        self.motor_states_rareRight = [data.motor_states[2].driver_voltage, data.motor_states[2].driver_temperature, data.motor_states[2].driver_state]


        # self.pubScvStatus.publish(scvMsgArr)

if __name__ == '__main__':
    rospy.init_node('Hunt', anonymous=True)
    # rospy.spin()

    while not rospy.is_shutdown():
        scvIn = usage()
        time.sleep(1)