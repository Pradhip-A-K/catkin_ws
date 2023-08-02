#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
import geometry_msgs.msg 
import moveit_msgs.msg



def main():
    rospy.init_node('moveit_example', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "manipulator"  # Replace this with the name of your robot's arm group
    move_group = MoveGroupCommander(group_name)

    #joint_goal = [-0.0025550133076528547, -1.9619211941671137, 1.3236743687936154, 0.6340849748953256, 1.5652427907212672, -1.567832590267542]
    #move_group.go(joint_goal, wait=True)
    # Define your target pose (modify the values accordingly)
    target_pose = geometry_msgs.msg.Pose()
    #target_pose.header.frame_id = "tool_gripper"
    target_pose.position.x = 0.3820367746036579
    target_pose.position.y = 0.11296966627627769
    target_pose.position.z = 0.35176450719646235
    target_pose.orientation.x = -6.748392586340887e-05
    target_pose.orientation.y = 0.705712521761222
    target_pose.orientation.z = 0.0020932625157863583
    target_pose.orientation.w = 0.7084952013439303


    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
