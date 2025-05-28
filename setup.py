from setuptools import setup
import os
from glob import glob

package_name = 'livox_lvx2_ros2_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS Developer',
    maintainer_email='rosdeveloper@example.com',
    description='A ROS 2 package to parse and publish Livox LVX v2.0.0.0 file data.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lvx2_publisher = livox_lvx2_ros2_publisher.livox_lvx2_ros2_publisher:main',
        ],
    },
)
