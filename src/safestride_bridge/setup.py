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
    zip_safe=True,
    maintainer='SafeStride maintainers',
    maintainer_email='maintainer@example.com',
    description=(
        'Fail-safe serial bridge between ROS 2 and the SafeStride controller.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = '
            'safestride_bridge.serial_bridge_node:main',
        ],
    },
)
