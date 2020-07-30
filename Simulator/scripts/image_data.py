#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
import cv2


def printImageData(data):
    
    cv_image = bridge.imgmsg_to_cv2(data,'bgr8')
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def getImageData():

    # Image Data Format
    # std_msgs/Header header
    #   uint32 seq
    #   time stamp
    #   string frame_id
    # uint32 height
    # uint32 width
    # string encoding
    # uint8 is_bigendian
    # uint32 step
    # uint8[] data

    rospy.init_node('imageData', anonymous=True)

    rospy.Subscriber("/aeroBITS/usb_cam/image_raw", Image, printImageData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    getImageData()