import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int64MultiArray
import io
import os
import sys
import tty
import termios


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motor_cmd_pub = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)

        # Subscribers
        self.encoder_sub = self.create_subscription(
            Int64MultiArray, 'encoder_ticks', self.encoder_callback, 10)

        # Control parameters
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.speed_step = 0.1
        self.turn_step = 0.5
        self.max_linear = 1.0
        self.max_angular = 2.0

        # Mode: 'vel' for cmd_vel, 'pwm' for direct PWM
        self.control_mode = 'vel'
        self.pwm_step = 50
        self.left_pwm = 0
        self.right_pwm = 0

        self.last_encoder = [0, 0]

        self._input_stream = None
        self._input_fd = None
        self._setup_input_stream()

        self.print_instructions()
        self.get_logger().info('Teleop Keyboard Node Started')

    def print_instructions(self):
        print("\n" + "=" * 60)
        print("ROS2 ARDUINO BRIDGE - KEYBOARD TELEOP")
        print("=" * 60)
        print("\nVelocity Control Mode (cmd_vel):")
        print("  W/S : Increase/Decrease linear speed")
        print("  A/D : Turn left/right")
        print("  X   : Stop")
        print("  SPACE : Emergency stop (all zeros)")
        print("\nPWM Control Mode (direct motor control):")
        print("  I/K : Increase/Decrease left motor PWM")
        print("  O/L : Increase/Decrease right motor PWM")
        print("  M   : Stop both motors")
        print("\nGeneral:")
        print("  T   : Toggle between velocity and PWM mode")
        print("  R   : Reset to zero")
        print("  Q   : Quit")
        print("=" * 60)
        print(f"\nCurrent Mode: {self.control_mode.upper()}")
        print(f"Linear: {self.linear_speed:.2f} m/s | Angular: {self.angular_speed:.2f} rad/s")
        print("=" * 60 + "\n")

    def encoder_callback(self, msg):
        """Display encoder values"""
        self.last_encoder = msg.data
        if self.control_mode == 'vel':
            print(f"\rEncoders: L={msg.data[0]:6d} R={msg.data[1]:6d} | "
                  f"Linear: {self.linear_speed:+.2f} m/s | Angular: {self.angular_speed:+.2f} rad/s  ",
                  end='', flush=True)
        else:
            print(f"\rEncoders: L={msg.data[0]:6d} R={msg.data[1]:6d} | "
                  f"PWM: L={self.left_pwm:+4d} R={self.right_pwm:+4d}  ",
                  end='', flush=True)

    def get_key(self):
        """Get single keypress"""
        if self._input_fd is None or self._input_stream is None:
            return None

        fd = self._input_fd
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = self._input_stream.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        """Main keyboard loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key is None:
                    self.get_logger().error(
                        'No TTY available for keyboard input. '
                        'Install xterm or run this node in a terminal.')
                    break

                key = key.lower()

                if self.control_mode == 'vel':
                    self.handle_velocity_mode(key)
                else:
                    self.handle_pwm_mode(key)

                # Common commands
                if key == 't':
                    self.control_mode = 'pwm' if self.control_mode == 'vel' else 'vel'
                    print(f"\n\nSwitched to {self.control_mode.upper()} mode\n")
                elif key == 'q':
                    print("\n\nQuitting...")
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            pass

    def handle_velocity_mode(self, key):
        """Handle velocity control commands"""
        if key == 'w':
            self.linear_speed = min(self.linear_speed + self.speed_step, self.max_linear)
        elif key == 's':
            self.linear_speed = max(self.linear_speed - self.speed_step, -self.max_linear)
        elif key == 'a':
            self.angular_speed = min(self.angular_speed + self.turn_step, self.max_angular)
        elif key == 'd':
            self.angular_speed = max(self.angular_speed - self.turn_step, -self.max_angular)
        elif key == 'x' or key == ' ':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        elif key == 'r':
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        if key in ['w', 's', 'a', 'd', 'x', ' ', 'r']:
            self.publish_cmd_vel()

    def handle_pwm_mode(self, key):
        """Handle direct PWM control commands"""
        if key == 'i':
            self.left_pwm = min(self.left_pwm + self.pwm_step, 255)
        elif key == 'k':
            self.left_pwm = max(self.left_pwm - self.pwm_step, -255)
        elif key == 'o':
            self.right_pwm = min(self.right_pwm + self.pwm_step, 255)
        elif key == 'l':
            self.right_pwm = max(self.right_pwm - self.pwm_step, -255)
        elif key == 'm' or key == ' ':
            self.left_pwm = 0
            self.right_pwm = 0
        elif key == 'r':
            self.left_pwm = 0
            self.right_pwm = 0

        if key in ['i', 'k', 'o', 'l', 'm', ' ', 'r']:
            self.publish_motor_cmd()

    def publish_cmd_vel(self):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(msg)

    def publish_motor_cmd(self):
        """Publish direct motor PWM command"""
        msg = Int32MultiArray()
        msg.data = [self.left_pwm, self.right_pwm]
        self.motor_cmd_pub.publish(msg)

    def _setup_input_stream(self):
        """Initialise a terminal stream for keyboard input"""
        try:
            fd = sys.stdin.fileno()
            if os.isatty(fd):
                self._input_stream = sys.stdin
                self._input_fd = fd
                return
        except (io.UnsupportedOperation, ValueError):
            pass

        try:
            tty_stream = open('/dev/tty')
            tty_fd = tty_stream.fileno()
            if os.isatty(tty_fd):
                self.get_logger().info('Keyboard input redirected to /dev/tty')
                self._input_stream = tty_stream
                self._input_fd = tty_fd
                return
            tty_stream.close()
        except OSError as exc:
            self.get_logger().warning(
                f'Unable to open /dev/tty for keyboard input: {exc}')

        self._input_stream = None
        self._input_fd = None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        if node._input_stream not in (None, sys.stdin):
            node._input_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
