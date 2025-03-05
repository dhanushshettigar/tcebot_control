#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import serial
import time

class Differential(Node):
    def __init__(self):
        super().__init__('differential')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('wheel_separation', 0.17)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Motor specs
        self.motor_rpm = 60
        self.max_pwm_val = 150
        self.min_pwm_val = 15  # Minimum PWM needed to move
        self.velocity_deadzone = 0.02  # Ignore very small velocities

        # Calculate max speed
        self.wheel_radius = self.wheel_diameter / 2
        self.max_speed = (2 * pi * self.wheel_radius * self.motor_rpm) / 60  # m/s

        # Serial communication
        try:
            self.ser = serial.Serial(self.port, 9600, timeout=1)
            self.ser.flush()
            self.get_logger().info(f"✅ Serial connected: {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to {self.port}: {e}")
            self.ser = None

        # Subscribe to velocity commands
        self.vel_subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Emergency stop timer
        self.last_cmd_time = time.time()
        self.create_timer(0.5, self.check_timeout)

    def send_to_arduino(self, lDIR, lPWM, rDIR, rPWM):
        """Send PWM and direction commands to the Arduino motor controller."""
        if self.ser:
            command = f"{int(lDIR)},{lPWM},{int(rDIR)},{rPWM}\n"
            self.ser.write(command.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info(f"📤 Sent to Arduino: {command.strip()}")

    def stop(self):
        """Stops the robot completely."""
        self.send_to_arduino(1, 0, 1, 0)  # Stop both wheels

    def wheel_vel_executer(self, left_speed, right_speed):
        """Converts velocity to PWM and sends to Arduino."""
        if abs(left_speed) < self.velocity_deadzone and abs(right_speed) < self.velocity_deadzone:
            self.stop()
            return

        # Scale PWM based on speed
        lPWM = max(min(((abs(left_speed) / self.max_speed) * self.max_pwm_val), self.max_pwm_val), 0)
        rPWM = max(min(((abs(right_speed) / self.max_speed) * self.max_pwm_val), self.max_pwm_val), 0)

        # Direction flags (True = forward, False = backward)
        lDIR = left_speed >= 0
        rDIR = right_speed >= 0

        self.send_to_arduino(lDIR, int(lPWM), rDIR, int(rPWM))

    def cmd_vel_callback(self, data):
        """Handles incoming velocity commands."""
        self.last_cmd_time = time.time()  # Reset timeout timer

        linear_vel = data.linear.x
        angular_vel = data.angular.z

        # Convert to wheel speeds
        VrplusVl = 2 * linear_vel
        VrminusVl = angular_vel * self.wheel_separation

        right_vel = (VrplusVl + VrminusVl) / 2
        left_vel = VrplusVl - right_vel

        self.wheel_vel_executer(left_vel, right_vel)

    def check_timeout(self):
        """Stops the robot if no command is received for 1 second."""
        if time.time() - self.last_cmd_time > 1.0:
            self.get_logger().warn("⏳ No command received! Stopping robot.")
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    differential_drive = Differential()
    rclpy.spin(differential_drive)
    differential_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()