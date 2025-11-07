#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, threading, time, errno, signal, atexit

PRIMARY_PORT  = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_5573232353035180F1C1-if00"
FALLBACK_PORT = "/dev/ttyACM1"

class FourWheelBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ---------- Params ----------
        self.declare_parameter('port', PRIMARY_PORT)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('cmd_timeout', 0.30)     # shorter watchdog
        self.declare_parameter('send_rate', 30.0)

        self.port        = self.get_parameter('port').value
        self.baud        = int(self.get_parameter('baud_rate').value)
        self.wheel_sep   = float(self.get_parameter('wheel_separation').value)
        self.max_lin     = float(self.get_parameter('max_linear').value)
        self.max_ang     = float(self.get_parameter('max_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.send_period = 1.0 / float(self.get_parameter('send_rate').value)

        # ---------- Serial open with retry ----------
        self.ser = self._open_serial_with_retry(self.port) or self._open_serial_with_retry(FALLBACK_PORT, note="fallback")
        if not self.ser:
            raise RuntimeError("Unable to open Arduino serial port (busy or missing).")
        time.sleep(2.0)  # give Arduino time to reset

        # Clear any bootloader garbage from Arduino reset
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.get_logger().info(f'Connected to Arduino on {self.port} @ {self.baud}')

        # ---------- State ----------
        self.lock = threading.Lock()
        self.last_cmd_time = self.get_clock().now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.stopped_once = True  # Start in stopped state

        # Send explicit STOP command on startup
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
            time.sleep(0.1)  # Give Arduino time to process
            self.get_logger().info('Sent initial STOP command to Arduino')
        except Exception as e:
            self.get_logger().warn(f'Failed to send initial STOP: {e}')

        # ---------- ROS I/O ----------
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.timer = self.create_timer(self.send_period, self.send_loop)

        # ---------- Ensure STOP on exit ----------
        atexit.register(self._safe_stop)
        signal.signal(signal.SIGINT, self._signal_stop_and_exit)
        signal.signal(signal.SIGTERM, self._signal_stop_and_exit)

        self.get_logger().info('FourWheelBridge up (no encoders)')

    # --- Serial helpers ---
    def _open_serial_with_retry(self, port, note="primary", retries=5, base_delay=0.4):
        for i in range(retries):
            try:
                ser = serial.Serial(port, self.baud, timeout=0.05, exclusive=True)
                self.port = port
                return ser
            except serial.SerialException as e:
                msg = str(e)
                busy = ("Device or resource busy" in msg) or getattr(e, 'errno', None) == errno.EBUSY
                self.get_logger().warn(f'{note} port "{port}" open failed ({msg}) attempt {i+1}/{retries}')
                time.sleep(base_delay * (i + 1))
                if not busy and "No such file or directory" in msg:
                    break  # no point retrying if device path wrong
        return None

    # --- ROS callbacks ---
    def cmd_vel_cb(self, msg: Twist):
        lin  = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        ang  = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        with self.lock:
            self.last_linear  = lin
            self.last_angular = ang
            self.last_cmd_time = self.get_clock().now()
            self.stopped_once = False  # we have fresh cmd

    def send_loop(self):
        with self.lock:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout:
                # Deadman: send STOP once, then do nothing
                if not self.stopped_once:
                    self._send_line("STOP\n")
                    self.stopped_once = True
                self.last_linear = 0.0
                self.last_angular = 0.0
                return
            # Keep streaming last command
            self._send_line(f"VEL,{self.last_linear:.4f},{self.last_angular:.4f}\n")

    # --- Low-level send ---
    def _send_line(self, s: str):
        try:
            self.ser.write(s.encode('utf-8'))
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # --- Safe stop paths ---
    def _safe_stop(self):
        try:
            self._send_line("STOP\n")
        except Exception:
            pass
        try:
            if getattr(self, 'ser', None) and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def _signal_stop_and_exit(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}; sending STOP and exiting')
        self._safe_stop()
        # allow launch to handle shutdown cleanly
        raise SystemExit

    def destroy_node(self):
        self._safe_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FourWheelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

