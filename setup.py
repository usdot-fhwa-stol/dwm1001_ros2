from setuptools import setup
from glob import glob
import os

package_name = 'dwm1001_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (',share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='TIERS',
    maintainer='volpe',
    maintainer_email='philip.azeredo@dot.gov',
    description='ROS2 package for Qorvo DWM1001',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dwm1001_ips = dwm1001_ros2.dwm1001_ros2_active:main",
            "dwm1001_ips_params = dwm1001_ros2.dwm1001_ros2_active_with_params:main",
            "dwm1001 = dwm1001_ros2.dwm1001_ros2:main",
            "dwm1001_heading_corrector = dwm1001_ros2.dwm1001_corrected_heading:main"
        ],
    },
)
