#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from example_interfaces.msg import String
from sensor_msgs.msg import LaserScan

from time import sleep

### LIDAR RANGE VALUE INDICES ###

     ######## FRONT ########
     ######## 0/450 ########
     ######
     ### 112,5     337,5 ###
     ######
     ########  225  ########
     ########  BACK ########
     
######## COUNTERCLOCKWISE #######

class lidarComms(Node):
    def __init__(self):
        super().__init__("lidar_comms_node")
        
        # creating compatible quality of service profile for laser scan data
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        # get data from lidar
        self.laser_subscriber = self.create_subscription(LaserScan, "/ldlidar_node/scan", self.handle_scan, qos_profile)
        
        # publish environment data based on computated lidar data
        self.environment_state = "path_blocked"
        self.publish_rate = 0.01
        
        self.environment_state_publisher = self.create_publisher(String, "environment_state", 10)
        self.publish_timer = self.create_timer(self.publish_rate, self.publish_state)
        
        self.get_logger().info("LIDAR COMMUNICATION SUCCESSFULLY INITIATED")
        
        
    def handle_scan(self, msg):        
        MIN_DISTANCE = 6.0

        try:
            # checking left of x axis
            for i in range(0, 22):
                if (msg.ranges[i] * 10) <= MIN_DISTANCE:
                    self.environment_state = "path_blocked"
                    return

            # checking right of x axis
            for i in range(430, 450):
                if (msg.ranges[i] * 10) <= MIN_DISTANCE:
                    self.environment_state = "path_blocked"
                    return
            
            self.environment_state = "path_clear"
                    
        except IndexError:
            # IndexError can occur because laser range[] index can varry between 445 - 455
            pass
              
        # throttling computation
        sleep(0.01)
        
    def publish_state(self):
        msg = String()
        msg.data = self.environment_state
        self.environment_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = lidarComms()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()