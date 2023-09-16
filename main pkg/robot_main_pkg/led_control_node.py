#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from custom_interfaces.msg import UltrasonicSensorStatus

import board
import busio
import adafruit_pca9685
from time import sleep


class ledControl(Node): 
    def __init__(self):
        super().__init__("led_control_node") 
        
        # initialize raspberry pi hat
        self.i2c = busio.I2C(board.SCL, board.SDA) 
        self.hat = adafruit_pca9685.PCA9685(self.i2c)

        # set board's PWM frequency
        self.hat.frequency = 60

        self.ledRed = self.hat.channels[14]
        self.ledBlue = self.hat.channels[15]
        
        # get environment state
        self.environment_state = "wait_for_init"
        self.environment_state_subscriber = self.create_subscription(String, "environment_state", self.controlLeds, 10)
        
        # get ultrasonic sensors state (0 = clear, 1 = blocked)
        self.left_wheel = 1
        self.right_wheel = 1
        self.back_wheel = 1
        
        self.ultrasonic_subscriber = self.create_subscription(UltrasonicSensorStatus, "ultrasonic_sensor_state", self.updateWheelStatus, 10)
        
        self.get_logger().info("LED CONTROL SUCCESSFULLY INITIATED")

    def controlLeds(self, msg):
        if msg.data == "path_blocked" or self.left_wheel == 1 or self.right_wheel == 1:
            self.ledRed.duty_cycle = 0xffff
            self.ledBlue.duty_cycle = 0xffff
        else:
            self.ledRed.duty_cycle = 0xffff
            self.ledBlue.duty_cycle = 0
            sleep(0.1)
            self.ledRed.duty_cycle = 0
            self.ledBlue.duty_cycle = 0xffff
            sleep(0.1)
            
    def updateWheelStatus(self, msg):
        self.left_wheel = msg.left_sensor
        self.right_wheel = msg.right_sensor
        self.back_wheel = msg.back_sensor


def main(args=None):
    rclpy.init(args=args)
    node = ledControl() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()