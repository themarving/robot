#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from custom_interfaces.msg import UltrasonicSensorStatus

import serial
import random
from time import sleep


class motorControl(Node):
    def __init__(self):
        super().__init__("motor_control_node")
        
        # initiate serial connection
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1.0)
        sleep(2)
        self.ser.reset_input_buffer()
        
        # get environment state
        self.environment_state = "wait_for_init"
        self.environment_state_subscriber = self.create_subscription(String, "environment_state", self.updateEnvironmentState, 10)
        
        # get ultrasonic sensors state (0 = clear, 1 = blocked)
        self.left_wheel = 1
        self.right_wheel = 1
        self.back_wheel = 1
        
        self.ultrasonic_subscriber = self.create_subscription(UltrasonicSensorStatus, "ultrasonic_sensor_state", self.updateWheels, 10)
        
        # send move orders to arduino
        # possible states: movingForwards, movingBackwards, turningLeft, turningRight, stopped 
        self.current_move_state = "stopped"
        
        self.comms_rate = 0.1
        self.arduino_comms_timer = self.create_timer(self.comms_rate, self.sendMoveOrder)
        
        self.get_logger().info("MOTOR CONTROL SUCCESSFULLY INITIATED")

    def updateEnvironmentState(self, msg):
        self.environment_state = msg.data
        
    def updateWheels(self, msg):
        self.left_wheel = msg.left_sensor
        self.right_wheel = msg.right_sensor
        self.back_wheel = msg.back_sensor
        
    def sendMoveOrder(self):
        if self.environment_state == "wait_for_init":
            self.get_logger().info("... WAITING FOR ENVIRONMENT STATE ...")
            return
        
        # possible orders (move states): movingForwards, movingBackwards, turningLeft, turningRight, stopped 
        if self.environment_state == "path_clear" and self.left_wheel == 0 and self.right_wheel == 0:
            # send string in bytes
            self.ser.write("movingForwards\n".encode('utf-8'))
            self.current_move_state = "movingForwards"
        else:
            # only randomly going left or right if we're not already turning 
            if self.current_move_state == "stopped" or self.current_move_state == "movingForwards":
                left = "turningLeft"
                right = "turningRight"

                # 50/50 chance of going left or right
                result = random.choice([left, right])
                self.current_move_state = result
    
                if result == "turningLeft":
                    self.ser.write("turningLeft\n".encode('utf-8'))
                else:
                    self.ser.write("turningRight\n".encode('utf-8'))

            else:
                if self.current_move_state == "turningLeft":
                    self.ser.write("turningLeft\n".encode('utf-8'))
                else:
                    self.ser.write("turningRight\n".encode('utf-8'))
        
    def getArduinoMoveState(self):
        self.ser.write("REQUEST_STATE\n".encode('utf-8'))
        sleep(1)
        
        # check if we have received data from arduino
        if self.ser.in_waiting > 0:
            # decode line into string
            arduinoState = self.ser.readline().decode('utf-8').rstrip()
        

def main(args=None):
    rclpy.init(args=args)
    node = motorControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()