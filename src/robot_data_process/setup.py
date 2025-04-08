from setuptools import setup, find_packages
from glob import glob

package_name = 'robot_data_process'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] + find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 node for SlushEngine communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_sync_moveit_node = robot_data_process.joint_sync_moveit_node:main',
        ],
    },
)