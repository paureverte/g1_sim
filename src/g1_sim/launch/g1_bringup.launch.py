from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_argument(context, launch_name, cli_name):
    value = LaunchConfiguration(launch_name).perform(context)
    if value == '':
        return []
    return [cli_name, value]


def _launch_simulator(context):
    arguments = [
        '--config', LaunchConfiguration('config').perform(context),
    ]
    arguments.extend(_optional_argument(context, 'unitree_mujoco_dir', '--unitree-mujoco-dir'))
    arguments.extend(_optional_argument(context, 'robot', '--robot'))
    arguments.extend(_optional_argument(context, 'scene', '--scene'))
    arguments.extend(_optional_argument(context, 'domain_id', '--domain-id'))
    arguments.extend(_optional_argument(context, 'network_interface', '--network-interface'))
    arguments.extend(_optional_argument(context, 'use_joystick', '--use-joystick'))
    arguments.extend(_optional_argument(context, 'joystick_type', '--joystick-type'))
    arguments.extend(_optional_argument(context, 'joystick_device', '--joystick-device'))
    arguments.extend(_optional_argument(context, 'joystick_bits', '--joystick-bits'))
    arguments.extend(_optional_argument(context, 'print_scene_information', '--print-scene-information'))
    arguments.extend(_optional_argument(context, 'enable_elastic_band', '--enable-elastic-band'))
    arguments.extend(_optional_argument(context, 'elastic_band_start_enabled', '--elastic-band-start-enabled'))
    arguments.extend(_optional_argument(context, 'elastic_band_point_z', '--elastic-band-point-z'))
    arguments.extend(_optional_argument(context, 'elastic_band_initial_length', '--elastic-band-initial-length'))

    return [
        Node(
            package='g1_sim',
            executable='unitree_mujoco_runner',
            name='unitree_mujoco_g1',
            output='screen',
            arguments=arguments,
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            },
        )
    ]


def _launch_rl_controller(context):
    arguments = [
        '--config', LaunchConfiguration('config').perform(context),
        '--auto-start', LaunchConfiguration('rl_auto_start').perform(context),
        '--fixstand-delay', LaunchConfiguration('rl_fixstand_delay').perform(context),
        '--velocity-delay', LaunchConfiguration('rl_velocity_delay').perform(context),
        '--cmd-vel-topic', LaunchConfiguration('cmd_vel_topic').perform(context),
        '--cmd-vel-timeout', LaunchConfiguration('policy_cmd_vel_timeout').perform(context),
    ]
    arguments.extend(_optional_argument(context, 'unitree_rl_lab_dir', '--unitree-rl-lab-dir'))
    arguments.extend(_optional_argument(context, 'network_interface', '--network-interface'))

    return [
        Node(
            package='g1_sim',
            executable='g1_rl_controller_runner',
            name='g1_rl_controller',
            output='screen',
            arguments=arguments,
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            },
            condition=IfCondition(LaunchConfiguration('use_rl_controller')),
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare('g1_sim')
    default_config = PathJoinSubstitution([package_share, 'config', 'g1_sim.yaml'])
    default_rviz_config = PathJoinSubstitution([package_share, 'config', 'g1_sim.rviz'])

    arguments = [
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_mujoco_ros_bridge', default_value='true'),
        DeclareLaunchArgument('use_rl_controller', default_value='true'),
        DeclareLaunchArgument('unitree_rl_lab_dir', default_value=EnvironmentVariable('UNITREE_RL_LAB_DIR', default_value='')),
        DeclareLaunchArgument('rl_auto_start', default_value='true'),
        DeclareLaunchArgument('rl_fixstand_delay', default_value='0.5'),
        DeclareLaunchArgument('rl_velocity_delay', default_value='4.0'),
        DeclareLaunchArgument('policy_cmd_vel_timeout', default_value='0.5'),
        DeclareLaunchArgument('state_joint_topic', default_value='g1/rl_joint_states'),
        DeclareLaunchArgument('base_pose_topic', default_value='g1/mujoco_base_pose'),
        DeclareLaunchArgument('root_frame', default_value='odom'),
        DeclareLaunchArgument('use_cmd_vel_bridge', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('cmd_vel_duration', default_value='0.2'),
        DeclareLaunchArgument('max_vx', default_value='1.0'),
        DeclareLaunchArgument('max_vy', default_value='0.3'),
        DeclareLaunchArgument('max_yaw', default_value='0.2'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('unitree_mujoco_dir', default_value=EnvironmentVariable('UNITREE_MUJOCO_DIR', default_value='')),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('scene', default_value=''),
        DeclareLaunchArgument('domain_id', default_value=''),
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('use_joystick', default_value=''),
        DeclareLaunchArgument('joystick_type', default_value=''),
        DeclareLaunchArgument('joystick_device', default_value=''),
        DeclareLaunchArgument('joystick_bits', default_value=''),
        DeclareLaunchArgument('print_scene_information', default_value=''),
        DeclareLaunchArgument('enable_elastic_band', default_value=''),
        DeclareLaunchArgument('elastic_band_start_enabled', default_value=''),
        DeclareLaunchArgument('elastic_band_point_z', default_value=''),
        DeclareLaunchArgument('elastic_band_initial_length', default_value=''),
    ]

    mujoco_ros_bridge = Node(
        package='g1_sim',
        executable='g1_mujoco_ros_bridge',
        name='g1_mujoco_ros_bridge',
        output='screen',
        arguments=[
            '--config', LaunchConfiguration('config'),
            '--unitree-mujoco-dir', LaunchConfiguration('unitree_mujoco_dir'),
            '--state-joint-topic', LaunchConfiguration('state_joint_topic'),
            '--base-pose-topic', LaunchConfiguration('base_pose_topic'),
            '--root-frame', LaunchConfiguration('root_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('use_mujoco_ros_bridge')),
    )

    cmd_vel_bridge = Node(
        package='g1_sim',
        executable='g1_cmd_vel_bridge',
        name='g1_cmd_vel_bridge',
        output='screen',
        arguments=[
            '--cmd-vel-topic', LaunchConfiguration('cmd_vel_topic'),
            '--duration', LaunchConfiguration('cmd_vel_duration'),
            '--max-vx', LaunchConfiguration('max_vx'),
            '--max-vy', LaunchConfiguration('max_vy'),
            '--max-yaw', LaunchConfiguration('max_yaw'),
        ],
        condition=IfCondition(LaunchConfiguration('use_cmd_vel_bridge')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_simulator), mujoco_ros_bridge, OpaqueFunction(function=_launch_rl_controller), cmd_vel_bridge, rviz])
