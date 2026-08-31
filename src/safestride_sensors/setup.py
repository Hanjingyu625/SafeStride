from setuptools import find_packages, setup

setup(
    name='safestride_sensors',
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/safestride_sensors'],
        ),
        ('share/safestride_sensors', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='SafeStride maintainers',
    maintainer_email='maintainer@example.com',
    description='SafeStride sensor adapters',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_node = safestride_sensors.gps_node:main',
        ],
    },
)
