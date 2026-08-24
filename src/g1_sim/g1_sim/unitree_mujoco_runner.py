import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


DEFAULT_UNITREE_MUJOCO_DIR = '/home/user/workspace/third_party/unitree_mujoco'
DEFAULT_MUJOCO_HOME = '/opt/mujoco/mujoco-3.3.6'
MUJOCO_RUNTIME_PATCH_VERSION = 'g1_sim_elastic_band_low_v1'


def _discover_unitree_mujoco_dir(configured_path):
    candidates = []
    if configured_path:
        candidates.append(Path(configured_path).expanduser())
    candidates.append(Path(DEFAULT_UNITREE_MUJOCO_DIR))
    candidates.append(Path.cwd() / 'third_party' / 'unitree_mujoco')

    for anchor in (Path.cwd(), Path(__file__).resolve()):
        candidates.extend(parent / 'third_party' / 'unitree_mujoco' for parent in anchor.parents)

    for candidate in candidates:
        if (candidate / 'simulate' / 'CMakeLists.txt').exists():
            return candidate.resolve()

    raise SystemExit(
        'unitree_mujoco directory not found. Set UNITREE_MUJOCO_DIR or pass '
        'unitree_mujoco_dir:=/path/to/unitree_mujoco to the launch file.'
    )


def _str_to_int(value):
    if isinstance(value, bool):
        return int(value)
    return int(value)


def _load_config(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    return config


def _write_unitree_config(config, unitree_mujoco_dir):
    simulate_dir = unitree_mujoco_dir / 'simulate'
    target = simulate_dir / 'config.yaml'
    target.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    return target


def _patch_file(path, replacements):
    content = path.read_text(encoding='utf-8')
    for old, new in replacements:
        if old not in content:
            raise SystemExit(f'Unable to patch {path}: expected snippet not found')
        content = content.replace(old, new)
    path.write_text(content, encoding='utf-8')


def _build_runtime_mujoco(simulate_dir, build_dir):
    env = os.environ.copy()
    env['CMAKE_PREFIX_PATH'] = ':'.join(path for path in [
        '/usr/local/lib/cmake',
        '/opt/unitree_robotics/lib/cmake',
        env.get('CMAKE_PREFIX_PATH', ''),
    ] if path)
    env['CPLUS_INCLUDE_PATH'] = ':'.join(path for path in [
        '/usr/local/include',
        '/usr/local/include/ddscxx',
        '/usr/local/include/iceoryx/v2.0.2',
        '/opt/unitree_robotics/include',
        '/opt/unitree_robotics/include/ddscxx',
        '/opt/unitree_robotics/include/iceoryx/v2.0.2',
        env.get('CPLUS_INCLUDE_PATH', ''),
    ] if path)
    env['LIBRARY_PATH'] = ':'.join(path for path in [
        '/usr/local/lib',
        '/opt/unitree_robotics/lib',
        env.get('LIBRARY_PATH', ''),
    ] if path)
    subprocess.run(['cmake', '-S', str(simulate_dir), '-B', str(build_dir), '-DCMAKE_BUILD_TYPE=Release'], check=True, env=env)
    subprocess.run(['cmake', '--build', str(build_dir), '--target', 'unitree_mujoco', f'-j{os.cpu_count() or 1}'], check=True, env=env)


def _patch_runtime_mujoco(simulate_dir):
    _patch_file(
        simulate_dir / 'src' / 'param.h',
        [
            (
                '    int enable_elastic_band;\n    int band_attached_link = 0;\n',
                '    int enable_elastic_band;\n    int elastic_band_start_enabled = 1;\n    double elastic_band_point_z = 3.0;\n    double elastic_band_initial_length = 0.0;\n    int band_attached_link = 0;\n',
            ),
            (
                '            enable_elastic_band = cfg["enable_elastic_band"].as<int>();\n',
                '            enable_elastic_band = cfg["enable_elastic_band"].as<int>();\n            if (cfg["elastic_band_start_enabled"]) elastic_band_start_enabled = cfg["elastic_band_start_enabled"].as<int>();\n            if (cfg["elastic_band_point_z"]) elastic_band_point_z = cfg["elastic_band_point_z"].as<double>();\n            if (cfg["elastic_band_initial_length"]) elastic_band_initial_length = cfg["elastic_band_initial_length"].as<double>();\n',
            ),
        ],
    )
    _patch_file(
        simulate_dir / 'src' / 'main.cc',
        [
            (
                '  param::config.band_attached_link = 6 * body_id;\n  \n  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;\n',
                '  param::config.band_attached_link = 6 * body_id;\n  elastic_band.point_[2] = param::config.elastic_band_point_z;\n  elastic_band.length_ = param::config.elastic_band_initial_length;\n  elastic_band.enable_ = param::config.elastic_band_start_enabled == 1;\n  \n  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;\n',
            ),
        ],
    )


def _prepare_runtime_tree(unitree_mujoco_dir, executable):
    runtime_root = Path(os.environ.get('G1_SIM_RUNTIME_DIR', '/tmp/g1_sim_unitree_mujoco'))
    runtime_simulate_dir = runtime_root / 'simulate'
    runtime_build_dir = runtime_simulate_dir / 'build'
    runtime_executable = runtime_build_dir / 'unitree_mujoco'
    stamp = runtime_root / '.g1_sim_mujoco_patch_version'

    if not runtime_executable.exists() or not stamp.exists() or stamp.read_text(encoding='utf-8').strip() != MUJOCO_RUNTIME_PATCH_VERSION:
        if runtime_simulate_dir.exists():
            shutil.rmtree(runtime_simulate_dir)
        shutil.copytree(
            unitree_mujoco_dir / 'simulate',
            runtime_simulate_dir,
            ignore=shutil.ignore_patterns('build', 'config.yaml', 'mujoco'),
        )
        _patch_runtime_mujoco(runtime_simulate_dir)

    for source, target in [
        (unitree_mujoco_dir / 'unitree_robots', runtime_root / 'unitree_robots'),
        (unitree_mujoco_dir / 'simulate' / 'mujoco', runtime_simulate_dir / 'mujoco'),
    ]:
        if target.is_symlink() or target.exists():
            if target.is_dir() and not target.is_symlink():
                shutil.rmtree(target)
            else:
                target.unlink()
        target.symlink_to(source, target_is_directory=True)

    if not runtime_executable.exists() or not stamp.exists() or stamp.read_text(encoding='utf-8').strip() != MUJOCO_RUNTIME_PATCH_VERSION:
        _build_runtime_mujoco(runtime_simulate_dir, runtime_build_dir)
        stamp.write_text(MUJOCO_RUNTIME_PATCH_VERSION + '\n', encoding='utf-8')

    return runtime_root, runtime_executable


def _ensure_mujoco_link(unitree_mujoco_dir):
    mujoco_home = Path(os.environ.get('MUJOCO_HOME', DEFAULT_MUJOCO_HOME))
    simulate_mujoco = unitree_mujoco_dir / 'simulate' / 'mujoco'
    if simulate_mujoco.is_symlink() and not simulate_mujoco.exists():
        simulate_mujoco.unlink()
    if mujoco_home.exists() and not simulate_mujoco.exists():
        simulate_mujoco.symlink_to(mujoco_home, target_is_directory=True)


def _build_config(args):
    config = _load_config(args.config)

    def add_override(key, value, caster=lambda item: item):
        if value not in (None, ''):
            config[key] = caster(value)

    add_override('robot', args.robot)
    add_override('robot_scene', args.scene)
    add_override('domain_id', args.domain_id, _str_to_int)
    add_override('interface', args.network_interface)
    add_override('use_joystick', args.use_joystick, _str_to_int)
    add_override('joystick_type', args.joystick_type)
    add_override('joystick_device', args.joystick_device)
    add_override('joystick_bits', args.joystick_bits, _str_to_int)
    add_override('print_scene_information', args.print_scene_information, _str_to_int)
    add_override('enable_elastic_band', args.enable_elastic_band, _str_to_int)
    add_override('elastic_band_start_enabled', args.elastic_band_start_enabled, _str_to_int)
    add_override('elastic_band_point_z', args.elastic_band_point_z, float)
    add_override('elastic_band_initial_length', args.elastic_band_initial_length, float)
    return config


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Prepare and launch unitree_mujoco for G1.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--unitree-mujoco-dir', default=os.environ.get('UNITREE_MUJOCO_DIR'))
    parser.add_argument('--robot')
    parser.add_argument('--scene')
    parser.add_argument('--domain-id')
    parser.add_argument('--network-interface')
    parser.add_argument('--use-joystick')
    parser.add_argument('--joystick-type')
    parser.add_argument('--joystick-device')
    parser.add_argument('--joystick-bits')
    parser.add_argument('--print-scene-information')
    parser.add_argument('--enable-elastic-band')
    parser.add_argument('--elastic-band-start-enabled')
    parser.add_argument('--elastic-band-point-z')
    parser.add_argument('--elastic-band-initial-length')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    unitree_mujoco_dir = _discover_unitree_mujoco_dir(args.unitree_mujoco_dir)
    executable = unitree_mujoco_dir / 'simulate' / 'build' / 'unitree_mujoco'

    if not executable.exists():
        raise SystemExit(
            'unitree_mujoco executable not found. Build it with: '
            f'cmake -S {unitree_mujoco_dir / "simulate"} '
            f'-B {unitree_mujoco_dir / "simulate" / "build"} -DCMAKE_BUILD_TYPE=Release && '
            f'cmake --build {unitree_mujoco_dir / "simulate" / "build"}'
        )

    _ensure_mujoco_link(unitree_mujoco_dir)
    runtime_root, runtime_executable = _prepare_runtime_tree(unitree_mujoco_dir, executable)
    config = _build_config(args)
    _write_unitree_config(config, runtime_root)

    env = os.environ.copy()
    library_paths = [
        str(unitree_mujoco_dir / 'simulate' / 'mujoco' / 'lib'),
        '/opt/unitree_robotics/lib',
        '/usr/local/lib',
        env.get('LD_LIBRARY_PATH', ''),
    ]
    env['LD_LIBRARY_PATH'] = ':'.join(path for path in library_paths if path)
    env.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    command = [
        str(runtime_executable),
        '--robot', str(config['robot']),
        '--scene', str(config['robot_scene']),
        '--domain_id', str(config['domain_id']),
        '--network', str(config['interface']),
    ]
    os.chdir(runtime_root / 'simulate')
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()
