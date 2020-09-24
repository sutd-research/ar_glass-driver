#!/usr/bin/python
import rospy
import sys
import numpy as np
import requests
import cv2

class HTTPImage():
    def __init__(self, server_id, port_id):
        self.server_id = server_id
        self.port_id = port_id

        self.url = 'http://'+server_id+":"+port_id+"/"
        rospy.loginfo("Connecting to Server at " + self.server_id + " on Port " + self.port_id)
        r =requests.get(self.url)
        rospy.loginfo(r.text)

    def get_image(self):
        """
            Requests for an image from the server, using HTTP
            Returns RGB8 image
        """

        r =requests.get(self.url+'messages') 

        # Convert 'File' into 'Image' (3D Array with BRG8 color encoding) format
        image = np.asarray(bytearray(r.content), dtype="uint8")
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        return image

    def send_image(self, image):
        """
            Sends an image to the server, using HTTP
            param: image    (RGB8 encoding)
        """  
        # Convert 'Image' (3D Array with BRG8 color encoding) into 'File' format
        _, image_file = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
        image_string = image_file.tostring()

        requests.post(self.url+'messages',data = image_string)







