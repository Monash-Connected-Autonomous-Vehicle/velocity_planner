import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'velocity_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owenbrooks',
    maintainer_email='owen.h.brooks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_planner = velocity_planner.velocity_planner:main',
            'fake_waypoint_publisher = velocity_planner.fake_waypoint_publisher:main',
            'fake_object_publisher = velocity_planner.fake_object_publisher:main',
            'waypoint_visualiser = velocity_planner.waypoint_visualiser:main',
            'object_visualiser = velocity_planner.object_visualiser:main',
            'waypoint_reader = velocity_planner.waypoint_reader:main',
            'simple_trapezoidal = velocity_planner.simple_trapezoidal:main',
            'pose_estimate_to_tf = velocity_planner.pose_estimate_to_tf:main',
        ],
    },
)
