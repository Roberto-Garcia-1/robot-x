from setuptools import find_packages, setup

package_name = 'robotx_ctrl'

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
    maintainer='Felipe Rivas',
    maintainer_email='rivascf@gmail.com',
    description='RobotX Control package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'buzzer_ctrl = robotx_ctrl.buzzer_node:init_node',
            'param_srv = robotx_ctrl.robotx_params:init_node',
            'arm_param_srv = robotx_ctrl.arm_ctrl_params:init_node',
            'arm_param_cte = robotx_ctrl.arm_param_srv_cte:init_node',
            'move_all = robotx_ctrl.move_all:init_node',
            'read_all = robotx_ctrl.read_all:init_node',
            'rosbag_recorder_desk = robotx_ctrl.rosbag_recorder_desk:init_node'              
        ],
    },
)
