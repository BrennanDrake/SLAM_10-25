from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'visual_slam_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files (if any)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brennan Drake',
    maintainer_email='81040749+BrennanDrake@users.noreply.github.com',
    description='Phase 4: Visual SLAM with RTAB-Map and RealSense D455',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feature_detector = visual_slam_ros.feature_detector_node:main',
        ],
    },
)
