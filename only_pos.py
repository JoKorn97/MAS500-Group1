#!/usr/bin/env python

import sys
import os
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import tf.transformations 

#Initialize moveit_commander
def main():
    im_num=0  
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_irb',anonymous=True)


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

print('sleeping')


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
print('sleeping')


move_group.clear_pose_targets()


if __name__ == '__main__':
    main()