from setuptools import find_packages, setup
setup(name='safestride_perception', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/safestride_perception']),('share/safestride_perception',['package.xml'])],
 install_requires=['setuptools'], zip_safe=True, maintainer='SafeStride maintainers',
 maintainer_email='maintainer@example.com', description='Surface perception policy', license='Apache-2.0')
