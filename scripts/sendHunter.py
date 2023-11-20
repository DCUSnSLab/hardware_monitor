#!/usr/env/bin python3
import time
import rospy
import std_msgs.msg
from std_msgs.msg import *
from hunter_msgs.msg import *

pub = rospy.Publisher('/hs', std_msgs.msg.Float64, queue_size=10)

def callback(data):
    print(data.battery_voltage)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.battery_voltage)
    voltage = data.battery_voltage
    pub.publish(voltage)
    #print(type(data.battery_voltage))
    #print(type(data))
    #pub.publish(data)
    return data.battery_voltage


def listener():
    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber("/hunter_status", HunterStatus, callback)
    pub = rospy.Publisher('/hs', HunterStatus, queue_size=10)

    while not rospy.is_shutdown():
        time.sleep(1)
        # pub.publish(callback)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass