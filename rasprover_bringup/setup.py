import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rasprover_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Donald',
    maintainer_email='your_email@example.com',
    description='Launch files and configurations for RaspRover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = rasprover_bringup.esp32_bridge:main',
            'motor_test = rasprover_bringup.motor_test:main',
        ],
    },
)