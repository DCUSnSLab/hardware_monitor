#!/usr/bin/env python3
import rospy
from hardware_monitor.srv import Logging
import os
import signal

class rosbagLoggingServer:
    # 시작 받으면 subprocess.Popen
    # stop 받으면 pro.pid 받아와서 kill하기
    def __init__(self):
        self.processPID = 0
        self.service = rospy.Service("/logging", Logging, self.handle_add_two_ints)

    def handle_add_two_ints(self, req):
        result = req.isLogging
        if str(result) == "LoggingStart":
            rospy.loginfo("LoggingStart")
            cmd = "rosbag record -a"
            os.system(cmd)  # returns the exit code in unix
            self.processPID = os.getpid()
            return str(result)

        elif str(result) == "LoggingStop":
            rospy.loginfo("LoggingStop")
            rospy.loginfo(self.processPID)
            os.kill(self.processPID, signal.SIGINT)
            return str(result)

# service call 특정 문자열을 받으면 rosbag record -a
# 종료

if __name__ == '__main__':
    rospy.init_node("Logging_Server")

    while not rospy.is_shutdown():
        rospy.loginfo("Logging Server start")
        handle = rosbagLoggingServer()
        rospy.loginfo("Service server has been started")

        rospy.spin()