from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rasprover_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Don',
    maintainer_email='dwilliestyle@gmail.com',
    description='OpenCV color tracking and object following for the RasPRover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_tracker = rasprover_tracking.color_tracker:main',
            'follow_controller = rasprover_tracking.follow_controller:main',
            'hsv_tuner = rasprover_tracking.hsv_tuner:main',
        ],
    },
)