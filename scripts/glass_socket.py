import rospy
import socket
import cv2
import numpy as np
import time
import json


class GlassSocket():
    """
            Communicator class using websocket for communication between AR Glass (Android) and PC (Python)
            param: host_ip   - String
            param: port      - Integer
    """
    def __init__(self, host_ip, port):
        self.HOST = host_ip
        self.PORT = port

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

    def get_image(self):
        """
            Requests for an image from the server, using websocket
            Returns RGB8 image
                        or None in case of communication failure
        """

        # Sending Image Get request using socket
        data = {}
        data['cmd'] = 'get_image'
        json_data = json.dumps(data)
        self.s.sendall(bytearray(json_data.encode()))

        # Waiting for data reception completion
        rospy.sleep(0.1)

        # Reading received data through socket
        data = self.s.recv(40960000)    # some large number
        try:
            image_arr = np.asarray(bytearray(data), dtype="uint8")
            image = cv2.imdecode(image_arr, cv2.IMREAD_COLOR)
            if ((image.shape[0] > 0) and (image.shape[1] > 0) and (image.shape[2] > 1) ):
                rospy.loginfo ("Received image: [%d, %d], bytes:%d" %(image.shape[0], image.shape[1], len(data)))
                return image
        except:
            rospy.logwarn("Image reception error")
            return None

    def send_bounding_box(self, leftx, topy, rightx, bottomy):
        """
            Sends bounding box coordinates to server, using websocket
            param: leftx
            param: topy
            param: rightx
            param: bottomy
        """  
        data = {}
        data['cmd'] = 'bounding_box'
        data['name'] = 'Item1'
        data['leftx'] = leftx
        data['topy'] = topy
        data['rightx'] = rightx 
        data['bottomy'] = bottomy 
        json_data = json.dumps(data)
        self.s.sendall(bytearray(json_data.encode()))
