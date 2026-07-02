import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'hardware_monitor2'

setup(
    name= package_name,
    version='0.1.0',
    packages= [package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeon',
    maintainer_email='yeon@todo.todo',
    description='ROS2 hardware monitor',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'send_hunter = hardware_monitor2.send_hunter:main',
            'add_two_ints = hardware_monitor2.add_two_ints:main',
        ],
    },
)
