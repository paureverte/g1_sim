from glob import glob
from setuptools import setup

package_name = 'g1_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='pau.reverte',
    maintainer_email='pau.reverte@example.com',
    description='Bringup package for the Unitree G1 MuJoCo simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'unitree_mujoco_runner = g1_sim.unitree_mujoco_runner:main',
            'g1_mujoco_ros_bridge = g1_sim.g1_mujoco_ros_bridge:main',
            'g1_cmd_vel_bridge = g1_sim.g1_cmd_vel_bridge:main',
            'g1_rl_controller_runner = g1_sim.g1_rl_controller_runner:main',
            'g1_mujoco_key = g1_sim.g1_mujoco_key:main',
        ],
    },
)
