#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from hunter_msgs.msg import *
import psutil
import nvidia_smi
import socket
from math import pi

class scvInformation:

    flag = 0
    pubCpu = rospy.Publisher('/pubCpu', Float64, queue_size=10)
    pubRam = rospy.Publisher('/pubRam', Float64, queue_size=10)
    pubGpu = rospy.Publisher('/pubGpu', Float64, queue_size=10)
    pubScvIP = rospy.Publisher('/pubScvIP', String, queue_size=10)

    def __init__(self):
        self.pubCpu.publish(self.cpuUsage())
        self.pubGpu.publish(self.gpuUsage())
        self.pubRam.publish(self.ramUsage())
        self.pubScvIP.publish(self.checkSCVIP())
        # self.hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, self.callback)

    def gpuUsage(self):
        nvidia_smi.nvmlInit()
        deviceCount = nvidia_smi.nvmlDeviceGetCount()
        for i in range(deviceCount):
            handle = nvidia_smi.nvmlDeviceGetHandleByIndex(i)
            util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
            mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            # print(
            #     f"|Device {i}| Mem Free: {mem.free / 1024 ** 2:5.2f}MB / {mem.total / 1024 ** 2:5.2f}MB "
            #     f"| gpu-util: {util.gpu / 100.0:3.1%} | gpu-mem: {util.memory / 100.0:3.1%} |")
            gpuUsageValue = util.gpu
            gpuMempercent = {util.memory / 100.0}
            # print(f"gpuUsageValue: {gpuUsageValue}")
            # print(f"gpuUsageValue: {gpuUsageValue}")
            return gpuUsageValue

    def cpuUsage(self):
        # self.cpuUsageValue = cpuUsageValue
        cpuUsageValue = psutil.cpu_percent()
        # print(f"CPU Percentage usage: {cpuUsageValue}%")
        # CPU frequency
        # cpu_info = psutil.cpu_freq()
        # print(f"CPU Current frequency: {cpuUsageValue}")
        return cpuUsageValue

    def ramUsage(self):
        ram_info = psutil.virtual_memory()
        # print(f"RAM Total: {ram_info.total / 1024 / 1024 / 1024:.2f} GB")
        # print(f"RAM Available: {ram_info.available / 1024 / 1024 / 1024:.2f} GB")
        # print(f"RAM Used: {ram_info.used / 1024 / 1024 / 1024:.2f} GB")
        #print(f"RAM Percentage usage: {ram_info.percent}%")
        ramTotalGB = {ram_info.total / 1024 / 1024 / 1024: .2}
        ramAvailableGB = {ram_info.available / 1024 / 1024 / 1024: .2}
        ramUsedGB = {ram_info.used / 1024 / 1024 / 1024}
        ramUsageValue = ram_info.percent
        # print(f"RAM Percentage usage: {ramUsageValue}%")
        return ramUsageValue

    def checkSCVIP(self):
        # self.ipAddressValue = ipAddressValue
        hostname = socket.gethostname()
        ipAddressValue = socket.gethostbyname(hostname)
        # print(hostname, ":", ipAddressValue)
        return ipAddressValue

if __name__ == '__main__':
    rospy.init_node('scvStatus', anonymous=True)

    while not rospy.is_shutdown():
        scvIn = scvInformation()
        time.sleep(1)
