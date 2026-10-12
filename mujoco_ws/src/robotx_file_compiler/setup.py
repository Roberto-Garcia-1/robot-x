from setuptools import find_packages, setup
from glob import glob

package_name = 'robotx_file_compiler'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + "/launch", glob("launch/*")),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='roberto',
  maintainer_email='garg.roberto@gmail.com',
  description='TODO: Package description',
  license='TODO: License declaration',
  extras_require={
    'test': [
        'pytest',
    ],
  },
  entry_points={
    'console_scripts': [
      "mujoco_file_transformer  = robotx_file_compiler.mujoco_file_transformer:main",
      "urdf_file_compiler       = robotx_file_compiler.urdf_file_compiler:main"
    ],
  },
)
