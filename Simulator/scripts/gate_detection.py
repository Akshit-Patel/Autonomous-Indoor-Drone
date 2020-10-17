#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
import cv2
import random as rng

rng.seed(12345)


def printImageData(data):
    
    cv_image = bridge.imgmsg_to_cv2(data,'bgr8')
    cv2.imshow("Image window", cv_image)
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    hsv_color1 = np.asarray([5, 100, 0])
    hsv_color2 = np.asarray([30, 255, 255])

    mask = cv2.inRange(img_hsv, hsv_color1, hsv_color2)
    mask = cv2.GaussianBlur(mask, (5,5), 0)

    canny_output = cv2.Canny(mask, 10, 20)    
    contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

    hull_list = []
    for i in range(len(contours)):
        hull = cv2.convexHull(contours[i])
        hull_list.append(hull)

    # Draw contours + hull results
    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        color = (0, 0, 256)
        if ((len(hull_list[i])) >= 6) or ((len(hull_list[i])) <= 3):
            continue

        hull_color = (256, 256, 0)
        #cv2.drawContours(cv_image, contours, i, color)
        cv2.drawContours(cv_image, hull_list, i, hull_color)

    cv2.imshow("Convex Hulls", cv_image)
    cv2.imshow("Mask", mask)
    cv2.waitKey(100)

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

    rospy.Subscriber("/aeroBITS_sensors/usb_cam/image_raw", Image, printImageData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    getImageData()
