#!/usr/bin/python
import rospy
import sys
import signal
from ar_glass.srv import Image as ImageSrv
from ar_glass.srv import ImageRequest, ImageResponse
from ar_glass.msg import BoundingBox
from sensor_msgs.msg import Image as ImageMsg 
from cv_bridge import CvBridge
from glass_socket import GlassSocket
import numpy as np
import cv2

capture_srv = None
receiver_sub = None
bridge = CvBridge()
seq_no = 0
glass_socket = None

def bb_receiver_cb(msg):
    """
        Receives bounding box coordinates from External publisher and sends them to AR Glass
        :param bounding_box: type-

        This callback function gets called each time an external publisher sends a bounding box,
        to be sent to AR Glass.
    """
    leftx = msg.left_x
    topy = msg.top_y
    rightx = msg.right_x
    bottomy = msg.bottom_y

    rospy.loginfo("Received Bounding box [%d, %d, %d, %d]" %(leftx, topy, rightx, bottomy))
    glass_socket.send_bounding_box(leftx, topy, rightx, bottomy)

def capture_image(req):
    """
        Captures an Image from AR Glass
        :param req: Service Request (null)
        :return     image
                    type-sensor_msgs.Image 
        
        This function gets called each time an external requestor requests
        an image from the AR Glass.
    """
    response = ImageResponse()
    image = glass_socket.get_image()
    if not image is None:
        response.image = create_image_msg(image)
    return response

def create_image_msg(image):
    """
        Create a ROS Image Message from a image
        :param image: image (brg8 color encoded)
        :return     image_msg   (sensor_msgs/Image type)

        This function will add current timestamp and sequence number data to the image message
    """
    global seq_no
    image_msg = ImageMsg()
    image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    # Adding Header Information
    image_msg.header.seq = seq_no
    image_msg.header.stamp = rospy.get_rostime()
    image_msg.header.frame_id = "ar_glass"

    seq_no += 1

    return image_msg

def ar_glass():
    rospy.init_node('ar_glass_driver')
    signal.signal(signal.SIGINT, terminate)

    global server_id, port_id
    capture_service_name = rospy.get_param(rospy.get_name()+'/capture_service', 'capture')
    image_topic_name = rospy.get_param(rospy.get_name()+"/image_subscriber_topic", 'image_receiver')
    server_id = rospy.get_param(rospy.get_name()+"/server", '192.168.0.177')
    port_id = rospy.get_param(rospy.get_name()+"/port", 9191)

    global glass_socket
    glass_socket = GlassSocket(server_id, port_id)

    global capture_srv, receiver_sub
    capture_srv = rospy.Service(capture_service_name, ImageSrv, capture_image)
    receiver_sub = rospy.Subscriber(image_topic_name, BoundingBox, bb_receiver_cb)

    rospy.spin()


def terminate(*args):
    print ("Node Terminated")
    receiver_sub.unregister()
    sys.exit()

if __name__ == '__main__':
    try:
        ar_glass()
    except rospy.ROSInterruptException:
        pass
    



    
