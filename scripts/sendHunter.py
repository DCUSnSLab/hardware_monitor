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


# class 예쁘게 만들기

class scvInformation():
    pubCpu = rospy.Publisher('/pubCpu', Float64, queue_size=10)
    pubRam = rospy.Publisher('/pubRam', Float64, queue_size=10)
    pubGpu = rospy.Publisher('/pubGpu', Float64, queue_size=10)
    pubScvIP = rospy.Publisher('/pubScvIP', String, queue_size=10)
    pubScvStatus = rospy.Publisher('/pubScvStatus', Float32MultiArray, queue_size=10)

    def __init__(self):
        self.pubCpu.publish(self.cpuUsage())
        self.pubGpu.publish(self.gpuUsage())
        self.pubRam.publish(self.ramUsage())
        self.pubScvIP.publish(self.checkSCVIP())
        self.hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, self.callback)

    def gpuUsage(self):
        nvidia_smi.nvmlInit()
        deviceCount = nvidia_smi.nvmlDeviceGetCount()
        for i in range(deviceCount):
            handle = nvidia_smi.nvmlDeviceGetHandleByIndex(i)
            util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
            mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            print(
                f"|Device {i}| Mem Free: {mem.free / 1024 ** 2:5.2f}MB / {mem.total / 1024 ** 2:5.2f}MB "
                f"| gpu-util: {util.gpu / 100.0:3.1%} | gpu-mem: {util.memory / 100.0:3.1%} |")
            gpuUsageValue = util.gpu
            gpuMempercent = {util.memory / 100.0}
            print(gpuUsageValue)
            return gpuUsageValue

    def cpuUsage(self):
        # self.cpuUsageValue = cpuUsageValue
        cpuUsageValue = psutil.cpu_percent()
        print(f"CPU Percentage usage: {cpuUsageValue}%")
        # CPU frequency
        # cpu_info = psutil.cpu_freq()
        # print(f"CPU Current frequency: {cpu_info.current:.2f} Mhz")
        return cpuUsageValue

    def ramUsage(self):
        ram_info = psutil.virtual_memory()
        print(f"RAM Total: {ram_info.total / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Available: {ram_info.available / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Used: {ram_info.used / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Percentage usage: {ram_info.percent}%")
        ramTotalGB = {ram_info.total / 1024 / 1024 / 1024: .2}
        ramAvailableGB = {ram_info.available / 1024 / 1024 / 1024: .2}
        ramUsedGB = {ram_info.used / 1024 / 1024 / 1024}
        ramUsageValue = ram_info.percent
        print(ramUsageValue)
        return ramUsageValue

    def checkSCVIP(self):
        # self.ipAddressValue = ipAddressValue
        hostname = socket.gethostname()
        ipAddressValue = socket.gethostbyname(hostname)
        print(hostname, ":", ipAddressValue)
        return ipAddressValue

    def callback(self, data):
        floatMsgArr = Float32MultiArray()
        floatMsgArr.data = [data.control_mode, data.base_state, data.park_mode,
            data.battery_voltage, data.linear_velocity, data.steering_angle]
        self.pubScvStatus.publish(floatMsgArr)


if __name__ == '__main__':
    rospy.init_node('scvStatus', anonymous=True)
    # rospy.spin()

    while not rospy.is_shutdown():
        scvIn = scvInformation()
        time.sleep(1)