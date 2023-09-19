#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import FaceCoordinates

import cv2
from time import sleep


class computerVision(Node):
    def __init__(self):
        super().__init__("computer_vision_node")
        
        self.videoInput = cv2.VideoCapture(0)
        
        # pre trained face classifier
        self.faceCascade = cv2.CascadeClassifier('/home/name/haarcascade_frontalface_default.xml')
        
        self.faceCenterX = 0
        self.faceCenterY = 0
        
        self.face_detection_publisher = self.create_publisher(FaceCoordinates, "face_detection", 10)
        
        # analying video input for faces
        self.detect_rate = 0.75
        self.detect_timer = self.create_timer(self.detect_rate, self.detectFace)
        
        # publishing center of registered face
        self.publish_rate = 0.5
        self.publish_timer = self.create_timer(self.publish_rate, self.publish_face_detection)

        self.get_logger().info("COMPUTER VISION SUCCESSFULLY INITIATED")
        
    def detectFace(self):
        ret, frame = self.videoInput.read()
        
        if not ret:
            self.get_logger().error("CAMERA INIT FAILED")
            return
        
        # converting video to grayscale for processing
        grayscaledImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # finding faces in grayscaled frame
        # (1.3 = scale factor (max 1.4), larger value = faster but less accurate)
        faces = self.faceCascade.detectMultiScale(grayscaledImage, 1.3, 5)
        
        # only tracking one face at a time
        if len(faces) > 0:
            # extract the coordinates of the first detected face
            x, y, w, h = faces[0]

            # drawing a rectangle around the face
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
                
            # determining center of face 
            self.faceCenterX = x + (w / 2)
            self.faceCenterY = y + (h / 2)
            
    def publish_face_detection(self):
        msg = FaceCoordinates()
        msg.face_x_coordinate = self.faceCenterX
        msg.face_y_coordinate = self.faceCenterY
        self.face_detection_publisher.publish(msg)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = computerVision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
