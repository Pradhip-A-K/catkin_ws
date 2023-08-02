#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
rospy.init_node('get_end_effector_pose', anonymous=True)
robot = moveit_commander.RobotCommander()
move_group = moveit_commander.MoveGroupCommander("manipulator")
# Get the current pose of the end effector
current_pose = move_group.get_current_pose().pose
# Access the position components
current_position = current_pose.position
x = current_position.x
y = current_position.y
z = current_position.z

# Access the orientation components (as a quaternion)
current_orientation = current_pose.orientation
qx = current_orientation.x
qy = current_orientation.y
qz = current_orientation.z
qw = current_orientation.w

print(current_pose)
end_effector_frame_id = move_group.get_end_effector_link()

print("End effector frame ID:", end_effector_frame_id)
rospy.spin()
'''import rospy
import moveit_commander

rospy.init_node('get_end_effector_frame_id', anonymous=True)
move_group = moveit_commander.MoveGroupCommander("your_robot_arm_group_name")

# Get the end effector link name (frame ID)
end_effector_frame_id = move_group.get_end_effector_link()

print("End effector frame ID:", end_effector_frame_id)'''
