#!/usr/bin/python
import rospy
import sys
import signal
from ar_glass.srv import Image as ImageSrv
from ar_glass.srv import ImageRequest, ImageResponse
from sensor_msgs.msg import Image as ImageMsg 
from cv_bridge import CvBridge
from server import RequestHandler
import numpy as np
import cv2

capture_srv = None
receiver_sub = None
bridge = CvBridge()
server = None
seq_no = 0

def image_receiver_cb(image_msg):
    """
        Receives Images from external publishers and Redirects them to AR Glass
        :param image: type- sensor_msgs.Image

        This callback function gets called each time an external publisher sends an image,
        to be sent to AR Glass.
    """
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    ##### Example Code Only ####
    rospy.loginfo("Received Processed Image. Image size: [%d, %d]" %(image.shape[0], image.shape[1]))
    #############################

    # TODO
    # http.send_image(image)

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

    ##### Example Code To Create a Dummy Image for Testing ####
    height = 100
    width = 100
    image = np.zeros((height,width,3), np.uint8)
    image[:,0:width//2] = (255,0,0)      
    image[:,width//2:width] = (0,255,0)
    ###########################################################

    # TODO
    # image = http.get_image()

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

    capture_service_name = rospy.get_param(rospy.get_name()+'/capture_service', 'capture')
    image_topic_name = rospy.get_param(rospy.get_name()+"/image_subscriber_topic", 'image_receiver')
    server_address = rospy.get_param(rospy.get_name()+"/server", '192.168.0.177')
    port_id = rospy.get_param(rospy.get_name()+"/port", 80)

    global capture_srv, receiver_sub
    capture_srv = rospy.Service(capture_service_name, ImageSrv, capture_image)
    receiver_sub = rospy.Subscriber(image_topic_name, ImageMsg, image_receiver_cb)

    # TODO
    # Add any other initializations here

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
    



    
