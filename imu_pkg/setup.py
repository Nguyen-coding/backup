from setuptools import find_packages, setup

package_name = 'imu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_pkg.imu_publisher:main',
            'imu_subscriber = imu_pkg.imu_subscriber:main',
            'publisher = imu_pkg.publisher:main',
            'subscriber = imu_pkg.subscriber:main',
        ],
    },
)
