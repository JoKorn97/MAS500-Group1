#!/usr/bin/env python
#from email.policy import default
import sys
import os
import copy
import rospy
import rospkg
#import tf
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
#sys.path.insert(1,'/home/josteink/catkin_build_ws/src/moveit')
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from sensor_msgs.msg import PointCloud2
import numpy as np
#import tf.transformations
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv2

#Initialize moveit_commander
def main():
    im_num=0  
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_irb',anonymous=True)

    #Instantiate a RobotCommander object. Provides information
    s = Sample_02()
    s.im_num = 0
    robot = moveit_commander.RobotCommander()

    #Instantiate a PlanningSceneInterface object. This provides a remote interface for getting,
    scene = moveit_commander.PlanningSceneInterface()

    #define the groupname 
    group_name = 'righty_tcp'
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id('RRT')
    #move_group.set_planning_time(10)    
    #move_group.set_goal_position_tolerance(0.001)
    #move_group.set_goal_orientation_tolerance(0.005)

    #define the Publisher 
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    print ("Taking first capture")
    #print (move_group.get_current_pose())
    #print (move_group.get_current_rpy())

    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal = move_group.get_random_pose()

    #First picture
    pose_goal.orientation.x = 0.353
    pose_goal.orientation.y = 0.612
    pose_goal.orientation.z = 0.353
    pose_goal.orientation.w = 0.612
    pose_goal.position.x = 3.76 #3.8
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83



    #Move robot to the first position
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()

    #CALL CAPTURE SERVICE
    time.sleep(0.5)
    s.capture()
    time.sleep(1.5)
    print('sleeping')

    move_group.clear_pose_targets()

    #Second picture
    pose_goal.orientation.x = 0.187
    pose_goal.orientation.y = 0.682
    pose_goal.orientation.z = 0.187
    pose_goal.orientation.w = 0.682
    pose_goal.position.x = 3.76 #3.8
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83



    #Move robot to the second position
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is45.363 no residual movement
    move_group.stop()
    rospy.sleep(2)

    #CALL CAPTURE SERVICE
    s.im_num=s.im_num+1
    time.sleep(0.5)
    s.capture()
    print('sleeping')
    time.sleep(1.5)

    move_group.clear_pose_targets()


    #Third capture
    #q = tf.transformations.quaternion_from_euler(0,1.5707963, 0) 
    pose_goal.orientation.x = 0#q[0]
    pose_goal.orientation.y = 0.707#q[1]
    pose_goal.orientation.z = 0#q[2]
    pose_goal.orientation.w = 0.707#q[3]
    pose_goal.position.x = 3.76 
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83 #0.74 

    #Move robot to the third position
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    rospy.sleep(2)

    #CALL CAPTURE SERVICE
    s.im_num=s.im_num+1
    time.sleep(0.5)
    s.capture()
    print('sleeping')
    time.sleep(1.5)

    move_group.clear_pose_targets()

    #Fourth capture
    pose_goal.orientation.x = -0.182
    pose_goal.orientation.y = 0.683
    pose_goal.orientation.z = -0.182
    pose_goal.orientation.w = 0.683
    pose_goal.position.x = 3.76 
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83 #0.74 

    #Move robot to the fourth position
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    rospy.sleep(2)

    #CALL CAPTURE SERVICE
    s.im_num=s.im_num+1
    time.sleep(0.5)
    s.capture()
    print('sleeping')
    time.sleep(1.5)

    move_group.clear_pose_targets()

    #Fifth capture
    pose_goal.orientation.x = -0.358
    pose_goal.orientation.y = 0.610
    pose_goal.orientation.z = -0.358
    pose_goal.orientation.w = 0.610
    pose_goal.position.x = 3.76 
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83 #0.74 

    #Move robot to the fifth position
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    rospy.sleep(2)

    #CALL CAPTURE SERVICE
    s.im_num=s.im_num+1
    time.sleep(0.5)
    s.capture()
    print('sleeping')
    time.sleep(1.5)

    move_group.clear_pose_targets()

    #Sixth capture
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0.378
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.926
    pose_goal.position.x = 3.76 
    pose_goal.position.y = 5.0
    pose_goal.position.z = 0.83 #0.74 

    #Move robot to the sixth position
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    rospy.sleep(2)

    #CALL CAPTURE SERVICE
    s.im_num=s.im_num+1
    time.sleep(0.5)
    s.capture()
    print('sleeping')
    time.sleep(1.5)

    move_group.clear_pose_targets()




#Zivid capture sample
class Sample_02:
    def __init__(self):
        #rospy.init_node("sample_capture_2d_py", anonymous=True)

    

        #Sample capture 2d
        rospy.loginfo("Starting sample_capture_2d.py")
        rospy.wait_for_service("/zivid_camera/capture_2d", 30.0)


    
        

        self.image_sub = rospy.Subscriber("/zivid_camera/color/image_color", Image, self.image_callback)
        #rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points)

       

        #Sample capture 2d
        self.capture_2d_service = rospy.ServiceProxy("/zivid_camera/capture_2d", Capture2D)

        self.bridge = CvBridge()

        

    

        #Sample capture 2d
        rospy.loginfo("Configuring 2D settings")


        
        #Sample capture 2d
        acquisition_0_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/settings_2d/acquisition_0"
        )
        
        #Sample capture 2d
        acquisition_0_config = {
            "enabled": True,
            "aperture": 2.83,
            "exposure_time": 10000,
            "brightness": 1.0,
        }

       

        acquisition_0_client.update_configuration(acquisition_0_config)

    def image_callback(self,data):
        print("Received an image!")
        try:
        # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            self.im_num = self.im_num+1
            directory_im = r'/home/josteink/MAS500G1/yolov5/data/LIBRES'
            im_name = str(self.im_num)+'.jpg'
            cv2.imwrite(os.path.join(directory_im, im_name), cv2_img)
            #cv2.waitkey(100)
            rospy.loginfo('Image saved')
            cv2.destroyAllWindows()
            
    
    def capture(self):
        rospy.loginfo("Calling capture service")
        #self.capture_service()
        self.capture_2d_service()

    #def on_points(self, data):
        #rospy.loginfo("PointCloud received")
        #self.capture()


if __name__ == '__main__':
    main()