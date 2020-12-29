#! /usr/bin/env python
#This code works for multiple waypoints
#Based of the code taken from: https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import atan2, pi, cos, sin, pi
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty  


#For the sprayer aspect of thorvald
from std_srvs.srv import Empty

class MoveClass:
    def __init__(self):


        '''
        FOR movebase server to work:
        #TO activate the movebase server
        >> roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true
        >> roslaunch uol_cmp9767m_tutorial move_base.launch
        '''
        #These poses are the starting position of the crop rows used by the NavBase library
        self.pose_start_points = [(-6,-2.9), (6,-1.9), (-6,0.1), (6,1.1), (-6,3),(6,4.1)]

        #These poses are the end positions for the simpler move control mechanism to have more control of thorvald
        self.pose_end_points = [(6,-2.9), (-6,-1.9), (6,0.1), (-6,1.1), (6,3),(-6,4.1)]

        #Both pose variables were found by hand as it was assumed that the position of the crop rows were known
        #For perception

        #The two up and down positions (z = 0, w = 1) for pointing up (X taken as north) and (z = 1, w = 0) for orientated South
        self.orientation = [(0,1), (1,0)]
        #A simple flag mechanism to tell thorvald whether it should point North or South depending on its position relative to the crop rows
        self.flag_as_up = True

        #Sets up the actionlib client so that MoveBase can be used
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


        #Commands to actually make it traverse the crop rows
        self.move = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)
        self.Coordinate = Twist()
        self.move_inpath = Pose()


        #Subscribes to the weeds "detected" flag to signal when to start spraying 
        self.plant_detect_flag_subscriber = rospy.Subscriber("plant_detect_flag_topic", String, self.weed_callback)
        

        #Connected to the Spray service provided by Sprayer.py
        self.spray_plant = rospy.ServiceProxy('spray', Empty)
        self.weed_detect = None


        #Subscribe to Odometry to allow for a simple proportional control mechanism used to traverse the crop rows
        self.final_pos_sub = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.pos_callback)

        #Simple flag mechanism that waits for the subscriptio to odometry to become active
        self.odometry_wait_flag = False

        #Waits for the move_base action server
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()

        #Simple code that closes connection to move base if program was prematurely exited
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return


        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Beginning to move to the start of the crop rows")
        
        
        #Function that makes thorvald move to the start of every crop row
        self.movebase_client()

    def weed_callback(self,data):
        #subscribe bbox sends the string message "weed" if weeds are detected
        #"no_weed" if there aren't any - so it's not constantly spraying weed
        self.weed_detect = str(data.data)


    def pos_callback (self, data):
        #Position callback connected to subscription to odometry
        #sets wait flag to true once connection to odometry has been established
        self.wait_flag = True
        self.move_inpath = data


    #Function with a very simple controller to move Thorvald along the crop rows
    def move_and_spray_row(self, coord):

        #Wait until there is subscription to odometry to prevent any errors
        while self.wait_flag == False:
            print("Just waiting for the subscriber on Odometry")

        #Finding the difference between where it is at and where it wants to go in cartesian coordinates
        difference_in_x = coord[0] - self.move_inpath.pose.pose.position.x
        difference_in_y = coord[1] - self.move_inpath.pose.pose.position.y

        #The rotation it needs to do to orientate itself towards the end of the crop rows
        angle_to_coord_euler = atan2(difference_in_y, difference_in_x)

        #Stop the control loop when Thorvald reaches the end of the crop rows
        while difference_in_x > 0.5 or difference_in_y > 0.5 or difference_in_x < -0.5 or difference_in_y < -0.5:

            #Gets the current orientation of thorvald which is given in quaternions
            current_quat = [self.move_inpath.pose.pose.orientation.x,self.move_inpath.pose.pose.orientation.y,
                                                self.move_inpath.pose.pose.orientation.z,self.move_inpath.pose.pose.orientation.w]

            #Converts the quaternions to euler angles
            current_angle = euler_from_quaternion(current_quat)[2]

            #Finds the difference in the angles to be used for the error correction
            difference_in_z_angle = angle_to_coord_euler - current_angle

            #Keeps the angle within a certain tolerance (0 and 2pi)
            if difference_in_z_angle > pi:
                difference_in_z_angle -= 2*pi

            #Speed to move Thor forwards using Twist
            forward_speed = 0.3
            self.Coordinate.angular.z = difference_in_z_angle
            self.Coordinate.linear.x = forward_speed

            #Recalculates the difference in x for error correction
            difference_in_x = coord[0] - self.move_inpath.pose.pose.position.x
            difference_in_y = coord[1] - self.move_inpath.pose.pose.position.y

            #Publish the twist commands to make thorvald move forward
            self.move.publish(self.Coordinate)

            #Command to say that if thorvald has stopped and the "weed" flag has been detected then spray!
            if (self.move_inpath.twist.twist.linear.x < 0.1) and (self.weed_detect == "weed"):
                rospy.loginfo("Thorvald has stopped, now spraying the weeds!")
                self.spray_plant()

            #Sleep the loop to prevent over sending of the commands - allows for periodical stopping of thorvald
            rospy.sleep(1)


        print("Now reached the end of the crop row, moving to to the next one!")


    def movebase_client(self):

        #Initialises the goal variable to be sent to MoveBase
        goal = MoveBaseGoal()
   
        #Iterate through the individual goal positions/orientations to make Thorvald move to the start positions of the crop rows
        for count, pose in enumerate(self.pose_start_points):
            
            #Establishing the header for the goal variable
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()          
            goal.target_pose.pose.position.x = pose[0]
            goal.target_pose.pose.position.y = pose[1]

            #Simple flag mechanism to determine which way Thorvald should be facing when it has moved to the start of the crop rows
            if self.flag_as_up:
                #If thorvald should point up
                goal.target_pose.pose.orientation.z = self.orientation[0][0]
                goal.target_pose.pose.orientation.w = self.orientation[0][1]

                #set the flag to false so that thorvald orientates south for the next crop row
                self.flag_as_up = False

            elif self.flag_as_up == False:
                goal.target_pose.pose.orientation.z = self.orientation[1][0]
                goal.target_pose.pose.orientation.w = self.orientation[1][1]

                #set the flag to True so that thorvald orientates North for the next crop row 
                self.flag_as_up = True

            #Send the goal to move_base server to make Thorvald move to desired positions
            self.client.send_goal(goal)

            #Wait until thorvald has moved to required position
            wait = self.client.wait_for_result()

            #Simple error checking
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")

            else:
                #rospy.loginfo("Thorvald will now traverse the current crop row to spray infesticide on the weeds")
                
                #This is the bit that sprays the robot
                '''The traversion of the crop rows were done separately due to unexpected paths when using navbase
                as well as allowing for more control of speed across the crop rows
                Could potentially be coupled with a robot arm to spray the weeds more accurately! '''
                self.move_and_spray_row(self.pose_end_points[count])
                
                continue

        print("Finished visiting all the required way points - Simulation has finished.")
        rospy.spin() #Equivalent to while not rospy is done etc.




if __name__ == '__main__':
    try:
        #Initialise the movement node to connect to the master node
        rospy.init_node('move_base_sequence')
        #Class for moving thorvald
        MoveClass()
    except rospy.ROSInterruptException:
        #When the robot reaches the final position. Close the node
        ros.loginfo("Navigation finished")
    

    '''The movement was purposedly split into two.
    NavBase was used to ensure Thorvald went to the start of the crop rows with high(er) accuracy at the correct orientation
    The simple move controller was used to allow for better control over the crop rows 
        such as periodical stopping so that the sprayer could spray '''
