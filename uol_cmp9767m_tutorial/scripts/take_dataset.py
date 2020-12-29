#!/usr/bin/env python

import rospy
import cv2
import os
from cv2 import namedWindow, cvtColor, imshow, resizeWindow
from cv2 import destroyAllWindows
from cv2 import COLOR_BGR2HSV, waitKey
from cv2 import blur, Canny
import numpy as np
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from uol_cmp9767_tutorial.srv import *
from std_srvs.srv import Empty
from time import sleep





class image_converter:

    def __init__(self):


        #convert between ros msgs to CV images
        self.bridge = CvBridge()

        #Subscribe to Thorvalds camera
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.image_callback)

        #Publish the green plant mask for viewing in RViz
        self.plant_pub = rospy.Publisher('green_plant_mask', Image, queue_size=1)
        
        #Count of the current image taken
        self.count = 0


    def image_callback(self, data):
        #Named window for the kiect video feed
        namedWindow("Thorvald Kinect Feed", cv2.WINDOW_NORMAL)
        
        #Windwo display for the mask 
        namedWindow("Mask", cv2.WINDOW_NORMAL)
        resizeWindow("Mask", 640,480)
        #namedWindow("Result", cv2.WINDOW_NORMAL)
        #resizeWindow("Result", 640, 480)

        #Determines the sensitivity to the colour green 
        # Based of: https://github.com/LCAS/teaching/blob/42132582719f52c6e941b2d7eb372715a530f856/cmp3103m-code-fragments/scripts/color_contours.py#L33-L35
        sensitivity = 65
        green = 60

        lower_color = np.array([green - sensitivity, 100, 50])
        upper_color = np.array([green + sensitivity, 255, 255])
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


        #hsv makes it easier to separate out the colour values
        #This converts the messages back into (cv) images

        #Converts the cv image into suitable form
        hsv = cvtColor(cv_image, COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        res = cv2.bitwise_and(cv_image,cv_image,mask=mask)

        #mask_pub = cvtColor(mask, )
        #Result of the mask
        res_pub = self.bridge.cv2_to_imgmsg(res, "passthrough")

        imshow("Thorvald Kinect Feed", cv_image)
        #resizeWindow("Thorvald Kinect Feed", 640, 480)
        imshow('Mask', mask)
        #imshow('Result', res)
        
        
        if mean(mask) > 1:
            #Takes a photo if the mean of the mask is within a certain value (e.g. there have been green objects detected in the Kinect Camera)
            #This is to stop it from contantly taking photos (such as the soil without any objects)
            '''
            rospy.loginfo(
            "I HAVE DETECTED PLANT!"
            )
            '''

            #Write to a pre-defined folder with image names as "image1.jpg" etc.
            x = cv2.imwrite("./dataset/image"+str(self.count)+".jpg", cv_image)

            #Add 1 to the counter for the next image
            self.count = self.count + 1
            sleep(1)

            #publish is for rviz
            self.plant_pub.publish(res_pub)
        waitKey(1)



if __name__ == '__main__':
    #Program to take pictures whenever the masks detects anything green
    #Best used with the teleop command key
    rospy.init_node('image_converter')
    ic = image_converter()
    rospy.spin()

    destroyAllWindows()