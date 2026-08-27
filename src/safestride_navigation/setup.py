from setuptools import find_packages, setup


package_name = 'safestride_navigation'


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
    description='GPS crosswalk approach and crossing-assist controller.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'crosswalk_controller = '
            'safestride_navigation.crosswalk_controller_node:main',
        ],
    },
)
