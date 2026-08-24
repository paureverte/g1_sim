import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


DEFAULT_UNITREE_RL_LAB_DIR = '/home/user/workspace/third_party/unitree_rl_lab'
ROBOT_CONTROLLER_DIR = Path('deploy/robots/g1_29dof')
DEFAULT_RUNTIME_DIR = '/tmp/g1_sim_unitree_rl_lab'
RUNTIME_PATCH_VERSION = 'g1_sim_ros_cmd_vel_v7'


CTRL_FSM_AUTO_PATCH = '''
inline bool g1_sim_auto_start_enabled()
{
    const char* value = std::getenv("G1_RL_AUTO_START");
    return value == nullptr || std::string(value) == "1" || std::string(value) == "true";
}

inline double g1_sim_auto_delay(const char* name, double default_value)
{
    const char* value = std::getenv(name);
    if(value == nullptr) return default_value;
    try { return std::stod(value); }
    catch(...) { return default_value; }
}

inline bool g1_sim_auto_start_consumed = false;
'''


ROS_CMD_VEL_OBSERVATION = '''
class G1SimRosCmdVel
{
public:
    G1SimRosCmdVel()
    {
        context_ = std::make_shared<rclcpp::Context>();
        int argc = 0;
        char** argv = nullptr;
        context_->init(argc, argv);

        rclcpp::NodeOptions node_options;
        node_options.context(context_);
        node_ = std::make_shared<rclcpp::Node>("g1_rl_cmd_vel_listener", node_options);

        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);

        std::string topic = std::getenv("G1_RL_CMD_VEL_TOPIC") ? std::getenv("G1_RL_CMD_VEL_TOPIC") : "/cmd_vel";
        sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            topic,
            10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_[0] = msg->linear.x;
                cmd_[1] = msg->linear.y;
                cmd_[2] = msg->angular.z;
                last_msg_time_ = std::chrono::steady_clock::now();
            }
        );
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("g1/rl_joint_states", 10);
        executor_->add_node(node_);
    }

    std::vector<float> command(YAML::Node cfg)
    {
        executor_->spin_some();
        publish_joint_state();
        auto timeout = env_value("G1_RL_CMD_VEL_TIMEOUT", 0.5);
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_msg_time_).count();
        if(elapsed > timeout) {
            cmd_ = {0.0f, 0.0f, 0.0f};
        }

        std::vector<float> out = cmd_;
        auto yaw_limit = env_value("G1_RL_CMD_VEL_YAW_LIMIT", 1.0);
        out[0] = std::clamp(out[0], cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
        out[1] = std::clamp(out[1], cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
        out[2] = std::clamp(out[2], static_cast<float>(-yaw_limit), static_cast<float>(yaw_limit));
        return out;
    }

private:
    void publish_joint_state()
    {
        if(!FSMState::lowstate) return;
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = node_->now();
        joint_state.name = joint_names_;
        joint_state.position.resize(joint_names_.size(), 0.0);
        for(size_t i = 0; i < joint_names_.size() && i < FSMState::lowstate->msg_.motor_state().size(); ++i) {
            joint_state.position[i] = FSMState::lowstate->msg_.motor_state()[i].q();
        }
        joint_state_pub_->publish(joint_state);
    }

    static double env_value(const char* name, double default_value)
    {
        const char* value = std::getenv(name);
        if(value == nullptr) return default_value;
        try { return std::stod(value); }
        catch(...) { return default_value; }
    }

    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    const std::vector<std::string> joint_names_ = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
    };
    std::vector<float> cmd_ = {0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
};

REGISTER_OBSERVATION(ros_cmd_vel_commands)
{
    static G1SimRosCmdVel cmd_vel;
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
    return cmd_vel.command(cfg);
}
'''


def _discover_unitree_rl_lab_dir(configured_path):
    candidates = []
    if configured_path:
        candidates.append(Path(configured_path).expanduser())
    candidates.append(Path(DEFAULT_UNITREE_RL_LAB_DIR))
    candidates.append(Path.cwd() / 'third_party' / 'unitree_rl_lab')

    for anchor in (Path.cwd(), Path(__file__).resolve()):
        candidates.extend(parent / 'third_party' / 'unitree_rl_lab' for parent in anchor.parents)

    for candidate in candidates:
        robot_dir = candidate / ROBOT_CONTROLLER_DIR
        if (robot_dir / 'CMakeLists.txt').exists() and (robot_dir / 'config' / 'config.yaml').exists():
            return candidate.resolve()

    raise SystemExit(
        'unitree_rl_lab directory not found. Set UNITREE_RL_LAB_DIR or pass '
        'unitree_rl_lab_dir:=/path/to/unitree_rl_lab to the launch file.'
    )


def _load_config(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def _patch_file(path, replacements):
    content = path.read_text(encoding='utf-8')
    for old, new in replacements:
        if old not in content:
            raise SystemExit(f'Unable to patch {path}: expected snippet not found')
        content = content.replace(old, new)
    path.write_text(content, encoding='utf-8')


def _prepare_runtime_tree(unitree_rl_lab_dir, runtime_dir):
    runtime_root = Path(runtime_dir)
    runtime_deploy_dir = runtime_root / 'deploy'
    runtime_robot_dir = runtime_deploy_dir / 'robots' / 'g1_29dof'
    source_deploy_dir = unitree_rl_lab_dir / 'deploy'
    source_robot_dir = unitree_rl_lab_dir / ROBOT_CONTROLLER_DIR
    stamp = runtime_root / '.g1_sim_patch_version'

    if stamp.exists() and stamp.read_text(encoding='utf-8').strip() == RUNTIME_PATCH_VERSION:
        return runtime_robot_dir

    if runtime_root.exists():
        shutil.rmtree(runtime_root)
    runtime_deploy_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_deploy_dir / 'include', runtime_deploy_dir / 'include', dirs_exist_ok=True)
    shutil.copytree(source_robot_dir, runtime_robot_dir, dirs_exist_ok=True, ignore=shutil.ignore_patterns('build'))

    thirdparty_link = runtime_deploy_dir / 'thirdparty'
    if thirdparty_link.is_symlink() or thirdparty_link.exists():
        if thirdparty_link.is_dir() and not thirdparty_link.is_symlink():
            shutil.rmtree(thirdparty_link)
        else:
            thirdparty_link.unlink()
    thirdparty_link.symlink_to(source_deploy_dir / 'thirdparty', target_is_directory=True)

    _patch_runtime_controller(runtime_robot_dir, runtime_deploy_dir)
    stamp.write_text(RUNTIME_PATCH_VERSION + '\n', encoding='utf-8')
    return runtime_robot_dir


def _patch_runtime_controller(runtime_robot_dir, runtime_deploy_dir):
    cmake_lists = runtime_robot_dir / 'CMakeLists.txt'
    _patch_file(
        cmake_lists,
        [
            (
                'find_package(yaml-cpp REQUIRED)\n',
                'find_package(yaml-cpp REQUIRED)\nfind_package(ament_cmake REQUIRED)\nfind_package(rclcpp REQUIRED)\nfind_package(geometry_msgs REQUIRED)\nfind_package(sensor_msgs REQUIRED)\n',
            ),
            (
                'add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})\nlink_libraries(${PROJECT_NAME}_lib)\n\nadd_executable(g1_ctrl main.cpp)',
                'add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})\nament_target_dependencies(${PROJECT_NAME}_lib rclcpp geometry_msgs sensor_msgs)\nlink_libraries(${PROJECT_NAME}_lib)\n\nadd_executable(g1_ctrl main.cpp)\ntarget_link_libraries(g1_ctrl ${PROJECT_NAME}_lib)\nament_target_dependencies(g1_ctrl rclcpp geometry_msgs sensor_msgs)',
            ),
        ],
    )

    ctrl_fsm = runtime_deploy_dir / 'include' / 'FSM' / 'CtrlFSM.h'
    _patch_file(
        ctrl_fsm,
        [
            (
                '#include <unitree/common/thread/recurrent_thread.hpp>\n',
                '#include <unitree/common/thread/recurrent_thread.hpp>\n#include <chrono>\n#include <cstdlib>\n',
            ),
            (
                'class CtrlFSM\n{',
                CTRL_FSM_AUTO_PATCH + '\nclass CtrlFSM\n{',
            ),
            (
                '        currentState = states[0];\n        currentState->enter();\n\n        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(\n',
                '        currentState = states[0];\n        currentState->enter();\n        state_enter_time_ = std::chrono::steady_clock::now();\n\n        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(\n',
            ),
            (
                '        // Check if need to change state\n        int nextStateMode = 0;\n        for(int i(0); i<currentState->registered_checks.size(); i++)\n        {\n            if(currentState->registered_checks[i].first())\n            {\n                nextStateMode = currentState->registered_checks[i].second;\n                break;\n            }\n        }\n\n        if(nextStateMode != 0 && !currentState->isState(nextStateMode))\n',
                '        // Check if need to change state\n        int nextStateMode = 0;\n        auto now = std::chrono::steady_clock::now();\n        double state_elapsed = std::chrono::duration<double>(now - state_enter_time_).count();\n        if(g1_sim_auto_start_enabled() && !g1_sim_auto_start_consumed)\n        {\n            if(currentState->getStateString() == "Passive" && state_elapsed >= g1_sim_auto_delay("G1_RL_FIXSTAND_DELAY", 0.5))\n            {\n                nextStateMode = FSMStringMap.right.at("FixStand");\n            }\n            else if(currentState->getStateString() == "FixStand" && state_elapsed >= g1_sim_auto_delay("G1_RL_VELOCITY_DELAY", 4.0))\n            {\n                nextStateMode = FSMStringMap.right.at("Velocity");\n                g1_sim_auto_start_consumed = true;\n            }\n        }\n\n        if(nextStateMode == 0)\n        {\n            for(int i(0); i<currentState->registered_checks.size(); i++)\n            {\n                if(currentState->registered_checks[i].first())\n                {\n                    nextStateMode = currentState->registered_checks[i].second;\n                    break;\n                }\n            }\n        }\n\n        if(nextStateMode != 0 && !currentState->isState(nextStateMode))\n',
            ),
            (
                '                    currentState = state;\n                    currentState->enter();\n',
                '                    currentState = state;\n                    currentState->enter();\n                    state_enter_time_ = std::chrono::steady_clock::now();\n',
            ),
            (
                '    std::shared_ptr<BaseState> currentState;\n    unitree::common::RecurrentThreadPtr fsm_thread_;\n',
                '    std::shared_ptr<BaseState> currentState;\n    std::chrono::steady_clock::time_point state_enter_time_;\n    unitree::common::RecurrentThreadPtr fsm_thread_;\n',
            ),
        ],
    )

    state_rl_base = runtime_robot_dir / 'src' / 'State_RLBase.cpp'
    _patch_file(
        state_rl_base,
        [
            (
                '#include <unordered_map>\n',
                '#include <unordered_map>\n#include <algorithm>\n#include <cstdlib>\n#include <chrono>\n#include <rclcpp/rclcpp.hpp>\n#include <geometry_msgs/msg/twist.hpp>\n#include <sensor_msgs/msg/joint_state.hpp>\n',
            ),
            (
                '}\n\n}\n\nState_RLBase::State_RLBase',
                '}\n\n' + ROS_CMD_VEL_OBSERVATION + '\n}\n\nState_RLBase::State_RLBase',
            ),
        ],
    )

    deploy_yaml = runtime_robot_dir / 'config' / 'policy' / 'velocity' / 'v0' / 'params' / 'deploy.yaml'
    _patch_file(
        deploy_yaml,
        [
            (
                '  velocity_commands:\n  # keyboard_velocity_commands:\n',
                '  # velocity_commands:\n  ros_cmd_vel_commands:\n',
            ),
        ],
    )


def _str_to_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _with_unitree_sdk_paths(env):
    include_paths = [
        '/opt/unitree_robotics/include',
        '/opt/unitree_robotics/include/ddscxx',
        '/opt/unitree_robotics/include/iceoryx/v2.0.2',
        '/usr/local/include',
        '/usr/local/include/ddscxx',
        '/usr/local/include/iceoryx/v2.0.2',
        env.get('CPLUS_INCLUDE_PATH', ''),
    ]
    library_paths = [
        '/opt/unitree_robotics/lib',
        '/usr/local/lib',
        env.get('LIBRARY_PATH', ''),
    ]
    runtime_library_paths = [
        '/opt/unitree_robotics/lib',
        '/usr/local/lib',
        env.get('LD_LIBRARY_PATH', ''),
    ]
    env['CPLUS_INCLUDE_PATH'] = ':'.join(path for path in include_paths if path)
    env['LIBRARY_PATH'] = ':'.join(path for path in library_paths if path)
    env['LD_LIBRARY_PATH'] = ':'.join(path for path in runtime_library_paths if path)
    return env


def _build_controller(robot_dir, build_dir):
    env = _with_unitree_sdk_paths(os.environ.copy())
    subprocess.run(
        ['cmake', '-S', str(robot_dir), '-B', str(build_dir), '-DCMAKE_BUILD_TYPE=Release'],
        check=True,
        env=env,
    )
    subprocess.run(
        ['cmake', '--build', str(build_dir), f'-j{os.cpu_count() or 1}'],
        check=True,
        env=env,
    )


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Prepare and launch Unitree RL Lab G1 controller.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--unitree-rl-lab-dir', default=os.environ.get('UNITREE_RL_LAB_DIR'))
    parser.add_argument('--network-interface')
    parser.add_argument('--auto-build', default=os.environ.get('G1_SIM_AUTOBUILD_RL_CONTROLLER', '1'))
    parser.add_argument('--runtime-dir', default=os.environ.get('G1_SIM_RL_RUNTIME_DIR', DEFAULT_RUNTIME_DIR))
    parser.add_argument('--auto-start', default=os.environ.get('G1_RL_AUTO_START', '1'))
    parser.add_argument('--fixstand-delay', default=os.environ.get('G1_RL_FIXSTAND_DELAY', '0.5'))
    parser.add_argument('--velocity-delay', default=os.environ.get('G1_RL_VELOCITY_DELAY', '4.0'))
    parser.add_argument('--cmd-vel-topic', default=os.environ.get('G1_RL_CMD_VEL_TOPIC', '/cmd_vel'))
    parser.add_argument('--cmd-vel-timeout', default=os.environ.get('G1_RL_CMD_VEL_TIMEOUT', '0.5'))
    parser.add_argument('--cmd-vel-yaw-limit', default=os.environ.get('G1_RL_CMD_VEL_YAW_LIMIT', '1.0'))
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    config = _load_config(args.config)
    unitree_rl_lab_dir = _discover_unitree_rl_lab_dir(args.unitree_rl_lab_dir)
    robot_dir = _prepare_runtime_tree(unitree_rl_lab_dir, args.runtime_dir)
    build_dir = robot_dir / 'build'
    executable = build_dir / 'g1_ctrl'

    if not executable.exists():
        if not _str_to_bool(args.auto_build):
            raise SystemExit(
                'g1_ctrl executable not found. Build it with: '
                f'cmake -S {robot_dir} -B {build_dir} -DCMAKE_BUILD_TYPE=Release && '
                f'cmake --build {build_dir}'
            )
        _build_controller(robot_dir, build_dir)

    network_interface = args.network_interface or config.get('interface', '')
    env = _with_unitree_sdk_paths(os.environ.copy())
    env['G1_RL_AUTO_START'] = '1' if _str_to_bool(args.auto_start) else '0'
    env['G1_RL_FIXSTAND_DELAY'] = str(args.fixstand_delay)
    env['G1_RL_VELOCITY_DELAY'] = str(args.velocity_delay)
    env['G1_RL_CMD_VEL_TOPIC'] = str(args.cmd_vel_topic)
    env['G1_RL_CMD_VEL_TIMEOUT'] = str(args.cmd_vel_timeout)
    env['G1_RL_CMD_VEL_YAW_LIMIT'] = str(args.cmd_vel_yaw_limit)
    library_paths = [
        '/opt/unitree_robotics/lib',
        '/usr/local/lib',
        str(unitree_rl_lab_dir / 'deploy' / 'thirdparty' / 'onnxruntime-linux-x64-1.22.0' / 'lib'),
        env.get('LD_LIBRARY_PATH', ''),
    ]
    env['LD_LIBRARY_PATH'] = ':'.join(path for path in library_paths if path)

    command = [str(executable)]
    if network_interface:
        command.extend(['--network', str(network_interface)])

    os.chdir(build_dir)
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()