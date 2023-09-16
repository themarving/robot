#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import UltrasonicSensorStatus

import RPi.GPIO as GPIO
from time import sleep


class ultrasonicSensors(Node): 
    def __init__(self):
        super().__init__("ultrasonic_sensors_node") 
        
        # 0 = clear, 1 = blocked
        self.left_sensor = 1
        self.right_sensor = 1
        self.back_sensor = 1

        # GPIO pins connected to arduino
        self.SENSOR_LEFT = 17
        self.SENSOR_RIGHT = 18
        self.SENSOR_BACK = 27

        GPIO.setmode(GPIO.BCM)

        # setting up arduino pins as input
        GPIO.setup(self.SENSOR_LEFT, GPIO.IN)
        GPIO.setup(self.SENSOR_RIGHT, GPIO.IN)
        GPIO.setup(self.SENSOR_BACK, GPIO.IN)

        GPIO.setwarnings(False)
        
        self.measure_rate = 0.01
        self.sensor_timer = self.create_timer(self.measure_rate, self.activate_sensors)
        
        self.sensor_publisher = self.create_publisher(UltrasonicSensorStatus, "ultrasonic_sensor_state", 10)
        
        self.get_logger().info("ULTRASONIC SENSOR ARRAY SUCCESSFULLY INITIATED")
        
    def activate_sensors(self):
        # getting values from sensors and updating
        self.left_sensor = GPIO.input(self.SENSOR_LEFT)
        self.right_sensor = GPIO.input(self.SENSOR_RIGHT)
        self.back_sensor = GPIO.input(self.SENSOR_BACK)
    
        # publishing sensor msg
        msg = UltrasonicSensorStatus()
        msg.left_sensor = self.left_sensor
        msg.right_sensor = self.right_sensor
        msg.back_sensor = self.back_sensor
        self.sensor_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ultrasonicSensors() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()