from setuptools import find_packages, setup

setup(name='safestride_sensors', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',
                   ['resource/safestride_sensors']),
                  ('share/safestride_sensors', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='SafeStride maintainers', maintainer_email='maintainer@example.com',
      description='SafeStride sensor adapters', license='Apache-2.0')
