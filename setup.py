from setuptools import setup

package_name = 'velocity_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'fake_waypoints = velocity_planner.fake_waypoint_publisher:main',
            'waypoint_visualiser = velocity_planner.waypoint_visualiser:main',
        ],
    },
)
