import glob
from setuptools import find_packages, setup

package_name = 'gsam2_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liyun',
    maintainer_email='you@example.com',
    description='ROS2 bridge to Grounded-SAM-2 via conda-env subprocess',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gsam2_segmentation_node = gsam2_ros.gsam2_segmentation_node:main',
        ],
    },
)
