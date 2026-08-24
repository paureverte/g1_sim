import argparse
import sys

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState


G1_NUM_MOTOR = 29
LEFT_HIP_PITCH = 0
LEFT_ANKLE_PITCH = 4
RIGHT_HIP_PITCH = 6
RIGHT_ANKLE_PITCH = 10
WAIST_YAW = 12
ARM_START = 15

STAND_TARGET = [
    -0.10, 0.00, 0.00, 0.30, -0.20, 0.00,
    -0.10, 0.00, 0.00, 0.30, -0.20, 0.00,
    0.00, 0.00, 0.00,
    0.20, 0.20, 0.00, 0.60, 0.00, 0.00, 0.00,
    0.20, -0.20, 0.00, 0.60, 0.00, 0.00, 0.00,
]


def _clamp(value, low, high):
    return max(low, min(high, value))


def _sign(value):
    return -1.0 if value < 0.0 else 1.0


class G1StandController(Node):
    def __init__(self, args):
        super().__init__('g1_stand_controller')
        self._topic_lowstate = args.lowstate_topic
        self._topic_lowcmd = args.lowcmd_topic
        self._ramp_duration = args.ramp_duration
        self._kp_leg = args.kp_leg
        self._kp_upper = args.kp_upper
        self._kd_leg = args.kd_leg
        self._kd_upper = args.kd_upper
        self._kp_waist = args.kp_waist
        self._kd_waist = args.kd_waist
        self._pitch_kp = args.pitch_kp
        self._pitch_kd = args.pitch_kd
        self._pitch_limit = abs(args.pitch_limit)
        self._pitch_correction_sign = _sign(args.pitch_correction_sign)
        self._start_time = None
        self._initial_position = None
        self._mode_machine = 5
        self._pitch = 0.0
        self._pitch_rate = 0.0

        self._lowcmd_pub = self.create_publisher(LowCmd, self._topic_lowcmd, 10)
        self._lowstate_sub = self.create_subscription(LowState, self._topic_lowstate, self._on_lowstate, 10)
        self._timer = self.create_timer(0.002, self._publish_command)
        self.get_logger().info(
            f'Publishing G1 stand LowCmd on {self._topic_lowcmd}; waiting for {self._topic_lowstate}'
        )

    def _on_lowstate(self, msg):
        self._mode_machine = int(msg.mode_machine)
        self._pitch = float(msg.imu_state.rpy[1])
        self._pitch_rate = float(msg.imu_state.gyroscope[1])
        if self._initial_position is None:
            self._initial_position = [msg.motor_state[index].q for index in range(G1_NUM_MOTOR)]
            self._start_time = self.get_clock().now()
            self.get_logger().info('Received first lowstate; starting stand posture ramp')

    def _publish_command(self):
        if self._initial_position is None or self._start_time is None:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        ratio = _clamp(elapsed / self._ramp_duration, 0.0, 1.0)

        command = LowCmd()
        command.mode_pr = 0
        command.mode_machine = self._mode_machine
        pitch_correction = self._pitch_correction_sign * _clamp(
            self._pitch_kp * self._pitch + self._pitch_kd * self._pitch_rate,
            -self._pitch_limit,
            self._pitch_limit,
        )

        for index in range(G1_NUM_MOTOR):
            motor = command.motor_cmd[index]
            target = (1.0 - ratio) * self._initial_position[index] + ratio * STAND_TARGET[index]
            if index in (LEFT_HIP_PITCH, RIGHT_HIP_PITCH):
                target += pitch_correction
            elif index in (LEFT_ANKLE_PITCH, RIGHT_ANKLE_PITCH):
                target -= pitch_correction
            motor.mode = 1
            motor.q = float(target)
            motor.dq = 0.0
            motor.tau = 0.0
            if index < WAIST_YAW:
                motor.kp = self._kp_leg
                motor.kd = self._kd_leg
            elif index < ARM_START:
                motor.kp = self._kp_waist
                motor.kd = self._kd_waist
            else:
                motor.kp = self._kp_upper
                motor.kd = self._kd_upper

        self._lowcmd_pub.publish(command)


def _parse_args(argv):
    parser = argparse.ArgumentParser(description='Publish a simple G1 low-level stand posture controller.')
    parser.add_argument('--lowstate-topic', default='lowstate')
    parser.add_argument('--lowcmd-topic', default='lowcmd')
    parser.add_argument('--ramp-duration', type=float, default=3.0)
    parser.add_argument('--kp-leg', type=float, default=120.0)
    parser.add_argument('--kp-waist', type=float, default=60.0)
    parser.add_argument('--kp-upper', type=float, default=30.0)
    parser.add_argument('--kd-leg', type=float, default=4.0)
    parser.add_argument('--kd-waist', type=float, default=2.0)
    parser.add_argument('--kd-upper', type=float, default=1.0)
    parser.add_argument('--pitch-kp', type=float, default=0.8)
    parser.add_argument('--pitch-kd', type=float, default=0.08)
    parser.add_argument('--pitch-limit', type=float, default=0.25)
    parser.add_argument('--pitch-correction-sign', type=float, default=1.0)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = G1StandController(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
