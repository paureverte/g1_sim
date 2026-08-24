import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from g1_sim.unitree_mujoco_runner import _discover_unitree_mujoco_dir, _load_config

try:
    from rclpy.node import Node
except ModuleNotFoundError:
    class Node:
        pass


def _floats(value, default):
    if value is None:
        return default
    return [float(item) for item in value.split()]


def _quat_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _quat_from_axis_angle(axis, angle):
    norm = math.sqrt(sum(item * item for item in axis))
    if norm == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / norm
    return [math.cos(half_angle), axis[0] * scale, axis[1] * scale, axis[2] * scale]


def _to_ros_quat(quat):
    return quat[1], quat[2], quat[3], quat[0]


def _mesh_resources(model_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    meshdir = root.find('compiler').get('meshdir', 'meshes') if root.find('compiler') is not None else 'meshes'
    mesh_root = model_path.parent / meshdir
    meshes = {}
    for mesh in root.findall('./asset/mesh'):
        name = mesh.get('name')
        filename = mesh.get('file')
        if name and filename:
            meshes[name] = (mesh_root / filename).resolve()
    return root, meshes


def _body_links(body, parent_frame, links):
    frame = body.get('name')
    if not frame:
        return

    joint = None
    for candidate in body.findall('joint'):
        if candidate.get('type') != 'free':
            joint = candidate
            break

    links.append({
        'parent': parent_frame,
        'child': frame,
        'pos': _floats(body.get('pos'), [0.0, 0.0, 0.0]),
        'quat': _floats(body.get('quat'), [1.0, 0.0, 0.0, 0.0]),
        'joint_name': joint.get('name') if joint is not None else None,
        'joint_axis': _floats(joint.get('axis'), [0.0, 0.0, 1.0]) if joint is not None else None,
    })

    for child in body.findall('body'):
        _body_links(child, frame, links)


def _geom_markers(body, meshes, markers, marker_id=0):
    from visualization_msgs.msg import Marker

    frame = body.get('name')
    if frame:
        for geom in body.findall('geom'):
            if geom.get('type') != 'mesh' or geom.get('mesh') not in meshes:
                continue

            marker = Marker()
            marker.header.frame_id = frame
            marker.ns = 'g1_mujoco_meshes'
            marker.id = marker_id
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = 'file://' + str(meshes[geom.get('mesh')])
            marker.mesh_use_embedded_materials = False
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = _floats(geom.get('pos'), [0.0, 0.0, 0.0])
            quat = _quat_multiply(
                _floats(geom.get('quat'), [1.0, 0.0, 0.0, 0.0]),
                _floats(geom.get('meshquat'), [1.0, 0.0, 0.0, 0.0]),
            )
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = _to_ros_quat(quat)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            rgba = _floats(geom.get('rgba'), [0.7, 0.7, 0.7, 1.0])
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            markers.append(marker)
            marker_id += 1

    for child in body.findall('body'):
        marker_id = _geom_markers(child, meshes, markers, marker_id)
    return marker_id


def _joint_names(root):
    names = []
    for joint in root.findall('.//joint'):
        if joint.get('type') == 'free':
            continue
        name = joint.get('name')
        if name:
            names.append(name)
    return names


def _model_from_config(config_path, unitree_mujoco_dir):
    config = _load_config(config_path)
    robot = config.get('robot', 'g1')
    scene = config.get('robot_scene', 'scene_29dof.xml')
    if '23' in scene:
        model = 'g1_23dof.xml'
    else:
        model = 'g1_29dof.xml'
    return unitree_mujoco_dir / 'unitree_robots' / robot / model


class G1VisualizationPublisher(Node):
    def __init__(self, model_path, state_joint_topic):
        from sensor_msgs.msg import JointState
        from tf2_ros import TransformBroadcaster
        from visualization_msgs.msg import MarkerArray

        super().__init__('g1_visualization_publisher')
        root, meshes = _mesh_resources(model_path)
        worldbody = root.find('worldbody')
        if worldbody is None:
            raise RuntimeError(f'No worldbody found in {model_path}')

        links = []
        markers = []
        for body in worldbody.findall('body'):
            _body_links(body, 'world', links)
            _geom_markers(body, meshes, markers)

        self._links = links
        self._joint_names = _joint_names(root)
        self._joint_positions = {name: 0.0 for name in self._joint_names}
        self._base_quat = None
        self._markers = MarkerArray(markers=markers)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._marker_pub = self.create_publisher(MarkerArray, 'g1/mujoco_markers', 10)
        self._state_joint_sub = self.create_subscription(JointState, state_joint_topic, self._on_joint_state, 10)
        self._timer = self.create_timer(0.02, self._publish)
        self.get_logger().info(f'Publishing G1 RViz markers and dynamic TF from {model_path}; listening to {state_joint_topic}')

    def _on_joint_state(self, msg):
        for index, name in enumerate(msg.name):
            if index < len(msg.position) and name in self._joint_positions:
                self._joint_positions[name] = float(msg.position[index])

    def _publish(self):
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import JointState

        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = self._joint_names
        joint_state.position = [self._joint_positions[name] for name in self._joint_names]
        self._joint_state_pub.publish(joint_state)

        transforms = []
        for link in self._links:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = link['parent']
            transform.child_frame_id = link['child']
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = link['pos']

            quat = link['quat']
            if link['child'] == 'pelvis' and self._base_quat is not None:
                quat = self._base_quat
            elif link['joint_name'] in self._joint_positions:
                quat = _quat_multiply(quat, _quat_from_axis_angle(link['joint_axis'], self._joint_positions[link['joint_name']]))
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = _to_ros_quat(quat)
            transforms.append(transform)

        self._tf_broadcaster.sendTransform(transforms)

        for marker in self._markers.markers:
            marker.header.stamp = stamp
        self._marker_pub.publish(self._markers)


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Publish ROS visualization topics for the G1 MuJoCo model.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--unitree-mujoco-dir')
    parser.add_argument('--model')
    parser.add_argument('--state-joint-topic', default='g1/rl_joint_states')
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    args = _parse_args(argv if argv is not None else sys.argv[1:])
    unitree_mujoco_dir = _discover_unitree_mujoco_dir(args.unitree_mujoco_dir)
    model_path = Path(args.model).expanduser().resolve() if args.model else _model_from_config(args.config, unitree_mujoco_dir)

    rclpy.init(args=None)
    node = G1VisualizationPublisher(model_path, args.state_joint_topic)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
