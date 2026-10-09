import glob
from setuptools import find_packages, setup

package_name = 'nero_mtc_task'

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
    description='MoveIt Task Constructor pick-and-place using real perception-detected poses',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtc_pick_and_place = nero_mtc_task.mtc_pick_and_place:main',
        ],
    },
)
