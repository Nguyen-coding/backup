from setuptools import find_packages, setup

package_name = 'hot'

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
    maintainer='cv25',
    maintainer_email='kim5537217@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'step_detector = hot.step_detection.step_detector_node:main',
            'node1 = hot.step_detection.node1:main',
            'node2 = hot.step_detection.node2:main',
            'node3 = hot.step_detection.node3:main',
            'node4 = hot.step_detection.node4:main',
            'line = hot.step_detection.line:main',
            'node5 = hot.step_detection.node5:main',
            'node6 = hot.step_detection.node6:main',

            'test = hot.step_detection.test:main',
            'server = hot.server:main',

        ],
    },
)
