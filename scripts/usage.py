#!/usr/bin/env python3
import psutil
from time import sleep
import nvidia_smi


def gpuUsage():
    nvidia_smi.nvmlInit()
    deviceCount = nvidia_smi.nvmlDeviceGetCount()
    for i in range(deviceCount):
        handle = nvidia_smi.nvmlDeviceGetHandleByIndex(i)
        util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
        mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
        print(
            f"|Device {i}| Mem Free: {mem.free / 1024 ** 2:5.2f}MB / {mem.total / 1024 ** 2:5.2f}MB "
            f"| gpu-util: {util.gpu / 100.0:3.1%} | gpu-mem: {util.memory / 100.0:3.1%} |")

def cpuUsage():
    cpu_percent = psutil.cpu_percent()
    print(f"CPU Percentage usage: {cpu_percent}%")
    # CPU frequency
    cpu_info = psutil.cpu_freq()
    print(f"CPU Current frequency: {cpu_info.current:.2f} Mhz")

def ramUsage():
    ram_info = psutil.virtual_memory()
    print(f"RAM Total: {ram_info.total / 1024 / 1024 / 1024:.2f} GB")
    print(f"RAM Available: {ram_info.available / 1024 / 1024 / 1024:.2f} GB")
    print(f"RAM Used: {ram_info.used / 1024 / 1024 / 1024:.2f} GB")
    print(f"RAM Percentage usage: {ram_info.percent}%")

while True:
    gpuUsage()
    cpuUsage()
    ramUsage()

    sleep(1.5)



# print(f'gpu: {gpuValue}%, gpu-mem: {res.memory}%')
# print('RAM memory % used:', psutil.virtual_memory()[2])
# print("RAM: ", psutil.virtual_memory().percent, "CPU: ", psutil.cpu_percent(0))
# print("=========================================================================================")
# print("=========================================================================================")