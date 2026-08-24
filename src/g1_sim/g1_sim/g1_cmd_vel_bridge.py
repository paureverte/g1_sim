import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


ROBOT_API_ID_LOCO_SET_VELOCITY = 7105


def _clamp(value, limit):
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


class G1CmdVelBridge(Node):
    def __init__(self, args):
        super().__init__('g1_cmd_vel_bridge')
        self._duration = args.duration
        self._max_vx = args.max_vx
        self._max_vy = args.max_vy
        self._max_yaw = args.max_yaw
        self._publisher = self.create_publisher(Request, args.request_topic, 10)
        self._subscription = self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd_vel, 10)
        self.get_logger().info(
            f'Bridging {args.cmd_vel_topic} to {args.request_topic} with Unitree G1 SetVelocity API'
        )

    def _on_cmd_vel(self, msg):
        vx = _clamp(msg.linear.x, self._max_vx)
        vy = _clamp(msg.linear.y, self._max_vy)
        yaw = _clamp(msg.angular.z, self._max_yaw)

        if not all(math.isfinite(value) for value in [vx, vy, yaw]):
            self.get_logger().warn('Ignoring cmd_vel with non-finite values')
            return

        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY
        request.header.policy.noreply = True
        request.parameter = json.dumps({
            'velocity': [vx, vy, yaw],
            'duration': self._duration,
        })
        self._publisher.publish(request)


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Bridge geometry_msgs/Twist cmd_vel to Unitree G1 sport API requests.')
    parser.add_argument('--cmd-vel-topic', default='cmd_vel')
    parser.add_argument('--request-topic', default='/api/sport/request')
    parser.add_argument('--duration', type=float, default=0.2)
    parser.add_argument('--max-vx', type=float, default=0.5)
    parser.add_argument('--max-vy', type=float, default=0.3)
    parser.add_argument('--max-yaw', type=float, default=0.8)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = G1CmdVelBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
