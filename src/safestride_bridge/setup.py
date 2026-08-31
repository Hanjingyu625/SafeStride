from setuptools import find_packages, setup


package_name = 'safestride_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='SafeStride maintainers',
    maintainer_email='maintainer@example.com',
    description=(
        'Fail-safe ROS 2 serial bridges for SafeStride controllers.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = '
            'safestride_bridge.serial_bridge_node:main',
            'terrain_bridge_node = '
            'safestride_bridge.terrain_bridge_node:main',
        ],
    },
)
