import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'redis_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sjjin',
    maintainer_email='sjjin9384@example.com',
    description='ROS 2 package for Redis communication, stream management, and list reading.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'redis_reader = redis_comm.redis_reader:main',
            'redis_producer = redis_comm.redis_producer:main',
            'redis_node = redis_comm.redis_node:main',
            'redis_gui = redis_comm.redis_gui:main',
        ],
    },
)

