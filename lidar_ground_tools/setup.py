from setuptools import find_packages, setup

package_name = 'lidar_ground_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sjjin9384',
    maintainer_email='sjjin9384@example.com',
    description='One-shot LiDAR ground-plane calibration and PointCloud2 bag correction tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_lidar_ground_once = lidar_ground_tools.calibrate_lidar_ground_once:main',
            'apply_lidar_calibration_to_bag = lidar_ground_tools.apply_lidar_calibration_to_bag:main',
            'extract_pcd_from_bag = lidar_ground_tools.extract_pcd_from_bag:main',
        ],
    },
)
