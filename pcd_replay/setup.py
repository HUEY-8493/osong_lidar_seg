from setuptools import setup

package_name = 'pcd_replay'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sjjin9384',
    maintainer_email='sjjin9384@example.com',
    description='Streaming PCD sequence player for ROS 2 PointCloud2',
    license='Apache-2.0',
    entry_points={'console_scripts': ['pcd_sequence_player = pcd_replay.pcd_sequence_player:main']},
)
