import os
from setuptools import setup

package_name = 'all_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'), ['launch/all_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Launch file for running urdf_tutorial, ydlidar, and imu_pkg together',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
