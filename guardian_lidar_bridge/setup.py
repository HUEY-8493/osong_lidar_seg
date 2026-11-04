from setuptools import find_packages, setup

package_name = "guardian_lidar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/guardian_lidar_bridge"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "guardian_lidar_bridge = guardian_lidar_bridge.bridge_node:main",
    ]},
)
