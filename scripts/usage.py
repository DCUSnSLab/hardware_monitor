#!/usr/bin/env python3
import psutil
import os

from tqdm import tqdm
from time import sleep

# you can calculate percentage of available memory
print(psutil.virtual_memory().available * 100 / psutil.virtual_memory().total)
# 20.8

print('memory % used:', psutil.virtual_memory()[2])
pid = os.getpid()
python_process = psutil.Process(pid)
memoryUse = python_process.memory_info()[0]/2.**30*1000  # memory use in GB...I think
print('memory use:', memoryUse)


with tqdm(total=100, desc='cpu%', position=1) as cpubar, tqdm(total=100, desc='ram%', position=0) as rambar:
    while True:
        rambar.n=psutil.virtual_memory().percent
        cpubar.n=psutil.cpu_percent()
        rambar.refresh()
        cpubar.refresh()
        sleep(0.5)