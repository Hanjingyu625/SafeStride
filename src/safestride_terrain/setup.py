from setuptools import find_packages, setup
setup(name='safestride_terrain', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/safestride_terrain']),('share/safestride_terrain',['package.xml'])],
 install_requires=['setuptools'], zip_safe=True, maintainer='SafeStride maintainers',
 maintainer_email='maintainer@example.com', description='Terrain safety policy', license='Apache-2.0')
