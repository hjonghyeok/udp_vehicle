from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'acslab_udp_interface'


setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='acs',
    maintainer_email='acs@example.com',

    description='ACSLAB ROS2 UDP Interface',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'udp_node = acslab_udp_interface.udp_node:main',
        ],
    },
)