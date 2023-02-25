#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg2_predefined_pose', anonymous=True)

        self._planning_group1 = "gripper"
        self._planning_group2 = "arm"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)

        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame1 = self._group1.get_planning_frame()
        self._eef_link1 = self._group1.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self._planning_frame2 = self._group2.get_planning_frame()
        self._eef_link2 = self._group2.get_end_effector_link()


        rospy.loginfo(
            '\033[94m' + "Planning Group1: {}".format(self._planning_frame1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link1: {}".format(self._eef_link1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Planning Group2: {}".format(self._planning_frame2) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link2: {}".format(self._eef_link2) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self,group, arg_pose_name):
        
        if(group==1):
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
            self._group1.set_named_target(arg_pose_name)
            plan = self._group1.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        elif(group==2):
            rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
            self._group2.set_named_target(arg_pose_name)
            plan = self._group2.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        
        
        
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose(1,"close")
        rospy.sleep(2)
        ur5.go_to_predefined_pose(1,"open")
        rospy.sleep(2)
        ur5.go_to_predefined_pose(2,"rest")
        rospy.sleep(2)

    del ur5


if __name__ == '__main__':
    main()
