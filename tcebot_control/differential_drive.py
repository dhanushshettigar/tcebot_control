#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import RPi.GPIO as GPIO
import time
from math import pi

# GPIO Pin Definitions
leftEn = 27  
rightEn = 24  
leftBackward = 17 
leftForward = 18  
rightForward = 22  
rightBackward = 23  

# Robot Parameters
motor_rpm = 60  # Max RPM of motor at full voltage
wheel_diameter = 0.065  # In meters
wheel_separation = 0.11  # In meters
max_pwm_val = 100  # 100 for Raspberry Pi, 255 for Arduino
min_pwm_val = 0  # Minimum PWM value to move the robot

wheel_radius = wheel_diameter / 2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel * motor_rpm) / 60 
max_ang = (max_speed * 2) / wheel_separation 

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBackward, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBackward, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)

class MotorController(Node):
    def __init__(self):
        super().__init__('tcebot_motor_controller')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.lpwm_pub = self.create_publisher(Int32, 'lpwm', 10)
        self.rpwm_pub = self.create_publisher(Int32, 'rpwm', 10)
        self.ldir_pub = self.create_publisher(Bool, 'ldir', 10)
        self.rdir_pub = self.create_publisher(Bool, 'rdir', 10)
        self.get_logger().info("TCEBOT Motor Controller Initialized")

    def wheel_vel_executer(self, left_speed, right_speed):
        lspeedPWM = max(min(((abs(left_speed) / max_speed) * max_pwm_val), max_pwm_val), min_pwm_val)
        rspeedPWM = max(min(((abs(right_speed) / max_speed) * max_pwm_val), max_pwm_val), min_pwm_val)

        pwmL.ChangeDutyCycle(lspeedPWM)
        pwmR.ChangeDutyCycle(rspeedPWM)
        
        self.lpwm_pub.publish(Int32(data=int(lspeedPWM)))
        self.rpwm_pub.publish(Int32(data=int(rspeedPWM)))
        
        GPIO.output(leftForward, left_speed >= 0)
        GPIO.output(leftBackward, left_speed < 0)
        GPIO.output(rightForward, right_speed >= 0)
        GPIO.output(rightBackward, right_speed < 0)
        
        self.ldir_pub.publish(Bool(data=(left_speed >= 0)))
        self.rdir_pub.publish(Bool(data=(right_speed >= 0)))

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        VrplusVl = 2 * linear_vel
        VrminusVl = angular_vel * wheel_separation

        right_vel = (VrplusVl + VrminusVl) / 2
        left_vel = VrplusVl - right_vel

        self.wheel_vel_executer(left_vel, right_vel)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
