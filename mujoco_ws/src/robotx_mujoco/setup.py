from setuptools import find_packages, setup
from glob import glob

package_name = 'robotx_mujoco'

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
    maintainer='robousr',
    maintainer_email='roberto.gar.1748@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mujoco_file_transformer   = robotx_mujoco.mujoco_file_transformer:main",
            "mujoco_control_position   = robotx_mujoco.mujoco_control_position:main",
            "mujoco_control_trajectory = robotx_mujoco.mujoco_control_trajectory:main"
        ],
    },
)
