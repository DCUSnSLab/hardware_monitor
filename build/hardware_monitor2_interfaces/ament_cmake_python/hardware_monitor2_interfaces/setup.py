from setuptools import find_packages
from setuptools import setup

setup(
    name='hardware_monitor2_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('hardware_monitor2_interfaces', 'hardware_monitor2_interfaces.*')),
)
