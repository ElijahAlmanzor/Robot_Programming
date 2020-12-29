#!/usr/bin/env python


#Importing modules needed to be able to run the code
import roslib, rospy, image_geometry, tf
import argparse
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


#Darknet is running on laptops RTX2060 (mobile) GPU to ensure (close to) real time processing of the images using CUDA
#Average of around 20 fps
#Special messages used by the darknet implementation of ROS
#ROS compatible darknet was obtained from: https://github.com/leggedrobotics/darknet_ros
from darknet_ros_msgs.msg import BoundingBoxes

class display_bbox:

    #Constructor function to initialise the display_bbox class
    def __init__(self):

        #CvBridge functions allows for the conversion of the image msg from thorvald's kinect camera into suitable image format for opencv (or the opposite way round)
        self.bridge = CvBridge()

        #Variable to hold the bounding boxes detected by the darknet YOLO CNN
        self.current_bboxes = None

        #Subscribe to the kinect topic to receive image msg data
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.image_callback)

        #Need to comment
        self.camera_model = None
        self.tf_listener = tf.TransformListener()


        #Publishes whenever thorvald detects a weed
        self.plant_detect_publisher = rospy.Publisher('plant_detect_flag_topic', String, queue_size=1)


        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.weed_flag = False
        
        #Subscribe to the bounding boxes outputted by the YOLO CNN whenever it finds cropa, cropb, cropc and weeds.
        #"roslaunch darknet_ros darknet_ros_thor.launch" needs to be running for this to work
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bbox_callback)

        #Publisher for viewing of the bounding boxes in RVIZ
        self.image_pub = rospy.Publisher('image_with_bbox', Image, queue_size=1)

    #Function that receives the message data from kinect
    def image_callback(self, data):
        
        #Creates a window for displaying the image data with the bounding boxes
        #cv2.namedWindow("Thorvald Kinect Feed", cv2.WINDOW_NORMAL)

        #Converts to a suitable bgr8 format for displaying with openCV
        cv_image =  self.bridge.imgmsg_to_cv2(data, "bgr8")

        #Obtains the centre of the image obtained from Kinect to be used for spraying weeds close to the centre of the camera
        #The sprayer was located to be at the same location as the camera
        #Not physically possible but its a placeholder for a better spraying mechanism (such as a specilised robot arm) 
        x_centre_cv_image = int(cv_image.shape[1]/2)
        y_centre_cv_image = int(cv_image.shape[0]/2)

        #cv2.resizeWindow("Thorvald Kinect Feed", 640,480)

        #A little code to stuff the node from crashing/bugging


        #Essentially waits until bounding boxes are produced by darknet before continuing with the rest of the function
        #Nothing will be displayed until Thorvald reaches the crop rows
        #if self.current_bboxes == None: 
        #    rospy.sleep(1)
        #    return
 

        self.weed_list = list()
        try:
            #For loop to display bounding boxes 
            for bbox in self.current_bboxes:

                #The bounding boxes are detected via their corners relative to the whole image
                #this finds where the centre of the bounding box is - really just for precise locationing of the crops and weeeds
                #e.g. if a robot arm + spray was incorporated to the code
                bbox_centre_x = int((bbox.xmin+bbox.xmax)/2)
                bbox_centre_y =  int((bbox.ymin+bbox.ymax)/2)

                #Converts the pixel coordinates to 3D coordinates relative to the camera
                which_pixel = (bbox_centre_x, bbox_centre_y)
                pixel_to_3Dray = self.camera_model.projectPixelTo3dRay(which_pixel)


                '''Only keeping this for the pose array implmentation for RViz'''
                #Just finds the euclidean distance from 3D coordinate to the camera frame
                distance_to_camera = math.sqrt(pixel_to_3Dray[0]**2 + pixel_to_3Dray[1]**2 + pixel_to_3Dray[2]**2)
                #Just specifies where it is wrt to the camera
                point_wrt_cam = PoseStamped()
                point_wrt_cam.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"
                #Specify that there is no rotation of the camera
                point_wrt_cam.pose.orientation.w = 1.0
                point_wrt_cam.pose.position.x = pixel_to_3Dray[0]
                point_wrt_cam.pose.position.y = pixel_to_3Dray[1]
                point_wrt_cam.pose.position.z = pixel_to_3Dray[2]

                #print("Point coordinates relative to the camera:")
                #print(point_wrt_cam.pose.position)

                #Code that transforms current weed coordinate to the baselink of the robot
                #Might change it to the odometry base, put it in an array wrt to the world coordinates - print out to text file at the end
                point_wrt_baselink = self.tf_listener.transformPose('thorvald_001/base_link', point_wrt_cam)
                #print("Point coordinates relative to robots base link")
                #print(point_wrt_baselink.pose.position) 

                point_wrt_baselink_print = "Position of plant wrt to the baselink of the robot: "+ str(point_wrt_baselink.pose.position.x)+ str(point_wrt_baselink.pose.position.y)+ str(point_wrt_baselink.pose.position.y)
                distance_to_camera_print = "Distance of plant wrt to the kinect camera: "+str(distance_to_camera)

                #Just plots the bounding box in the image figure/window
                cv2.rectangle(cv_image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255, 0 ,0), 2)
                cv2.circle(cv_image, (bbox_centre_x,bbox_centre_y), 5, (0,255,0),5)
                cv2.putText(cv_image, bbox.Class, (bbox.xmin + 5, bbox.ymin -45), cv2.FONT_HERSHEY_COMPLEX_SMALL, 3, (0, 0 ,0), 2)
                cv2.putText(cv_image, str(round(bbox.probability,2)), (bbox.xmin + 5, bbox.ymin + 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 0 ,0), 2)

                #Could just spray from this node, but want to only spray when thorvald has stopped
                #As it then allows for robotic arm spraying of the weeds in future work
                #At the moment, if it is detecting weed, just spray
                #But now I want to only append when it's at the centre of the camera

                if bbox.Class == 'weed':# and bbox.probability > 0.6:
                    #weed_pixel_distance_to_centre = math.sqrt((x_centre_cv_image-bbox_centre_x)**2 + (y_centre_cv_image-bbox_centre_y)**2)
                    x_pixel_difference = abs(x_centre_cv_image-bbox_centre_x)
                    y_pixel_difference = abs(y_centre_cv_image-bbox_centre_y)
                    #if weed_pixel_distance_to_centre < 100:
                    if x_pixel_difference < 150 and y_pixel_difference < 150:
                        #print("Distance to centre of camera",x_pixel_difference,y_pixel_difference)
                        self.weed_list.append(bbox.Class)

                #cv2.putText(cv_image, distance_to_camera_print, (bbox.xmin + 5, bbox.ymin + 25), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0 ,0), 4)
                #cv2.putText(cv_image, point_wrt_baselink_print, (bbox.xmin + 5, bbox.ymin + 15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0 ,0), 4)


        except:
            #Just a placeholder
            x = 1
            #self.current_bboxes = None
            #No bounding boxes detected just yet


        weed = 'weed'
        if weed in self.weed_list:
            
            #rospy.loginfo("Thorvald has detected some weeds!")
            #Only publish this when there's a weed right under the camera
            self.plant_detect_publisher.publish("weed")
        else:
           
            self.plant_detect_publisher.publish("no_weed")
        
        #reset the list
        #self.weed_list = None


        image_to_publish = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.image_pub.publish(image_to_publish)
        #cv2.imshow("Thorvald Kinect Feed", cv_image)
        #cv2.waitKey(1)
    
    def bbox_callback(self, data):
        #Stores the current detected bounding boxes detected by YOLO
        self.current_bboxes = data.bounding_boxes
        #print(data)
        #rospy.loginfo(data.bounding_boxes[0].probability)



    #Leave this little code for now in case I want to store the coordinates of the weeds - then print them or store it in a frame of itself? Maybe? Viewable by Rvis
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once  



if __name__ == '__main__':

    #Initialise the node for connection (and name ID) for the ros master node
    rospy.init_node('bbox_display', anonymous=True)

    #Display the bounding boxes on a separate figure box/RVIZ
    bbox = display_bbox()

    #Makes the code loop (forever)
    rospy.spin()
    