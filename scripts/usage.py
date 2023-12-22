#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import *
from hunter_msgs.msg import *

class usage:
    pubScvStatus = rospy.Publisher('/pubScvStatus', Float64MultiArray, queue_size=10)

    def __init__(self):
        self.hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, self.callback)
        self.scvMsgArr = Float64MultiArray()

        # self.base_state = 0
        # self.battery_voltage = 0
        # self.control_mode = 0
        # self.fault_code = 0
        # self.linear_velocity = 0
        # self.park_mode = 0
        # self.steering_angle = 0
        #
        # self.driver_states_front = []
        # self.driver_states_rareLeft = []
        # self.driver_states_rareRight = []
        #
        #
        # self.motor_states_front = []
        # self.motor_states_rareLeft = []
        # self.motor_states_rareRight = []
        #
        # self.motorStates = [self.motor_states_front, self.motor_states_rareLeft, self.motor_states_rareRight]
        # self.driverStates = [self.driver_states_front, self.driver_states_rareLeft, self.motor_states_rareRight]
        self.driver_states_arr = [[0,0,0], [0,0,0], [0,0,0]]
        self.motor_states_arr = [[0,0,0,0], [0,0,0,0], [0,0,0,0]]

    def callback(self, data):
        # motor_states_front = [data.motor_states[0].current, data.motor_states[0].rpm, data.motor_states[0].temperature, data.motor_states[0].motor_pose]
        # motor_states_rareLeft = [data.motor_states[1].current, data.motor_states[1].rpm, data.motor_states[1].temperature, data.motor_states[1].motor_pose]
        # motor_states_rareRight = [data.motor_states[2].current, data.motor_states[2].rpm, data.motor_states[2].temperature, data.motor_states[2].motor_pose]

        for i in range(3):
            self.driver_states_arr[i] = [data.driver_states[i].driver_voltage, data.driver_states[i].driver_temperature, data.driver_states[i].driver_state]
            self.motor_states_arr[i] = [data.motor_states[i].current, data.motor_states[i].rpm, data.motor_states[i].temperature, data.motor_states[i].motor_pose]

        # driver_states_front = [data.driver_states[0].driver_voltage, data.driver_states[0].driver_temperature, data.driver_states[0].driver_state]
        # driver_states_rareLeft = [data.driver_states[1].driver_voltage, data.driver_states[1].driver_temperature, data.driver_states[1].driver_state]
        # driver_states_rareRight = [data.driver_states[2].driver_voltage, data.driver_states[2].driver_temperature, data.driver_states[2].driver_state]

        # motorStates = [motor_states_front, motor_states_rareLeft, motor_states_rareRight]
        # driverStates = [self.driver_states_front, self.driver_states_rareLeft, self.driver_states_rareRight]

        self.scvMsgArr.data = [data.base_state, data.battery_voltage, data.control_mode, data.fault_code, data.linear_velocity,
                               data.park_mode, data.steering_angle, self.motor_states_arr, self.driver_states_arr]
        print(self.scvMsgArr)
        self.pubScvStatus.publish(self.scvMsgArr)

if __name__ == '__main__':
    rospy.init_node('Hunt', anonymous=True)
    # rospy.spin()

    while not rospy.is_shutdown():
        scvIn = usage()
        time.sleep(5)