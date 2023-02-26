#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


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


    def set_joint_angles(self,group, arg_list_joint_angles):
        if(group==1):
            list_joint_values = self._group1.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            self._group1.set_joint_value_target(arg_list_joint_angles)
            self._group1.plan()
            flag_plan = self._group1.go(wait=True)

            list_joint_values = self._group1.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            pose_values = self._group1.get_current_pose().pose
            rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            rospy.loginfo(pose_values)
        elif group==2:
            list_joint_values = self._group2.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            self._group2.set_joint_value_target(arg_list_joint_angles)
            self._group2.plan()
            flag_plan = self._group2.go(wait=True)

            list_joint_values = self._group2.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            pose_values = self._group2.get_current_pose().pose
            rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            rospy.loginfo(pose_values)
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

def main():

    ur5 = Ur5Moveit()
    mid1 = [math.radians(92),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    mid2 = [math.radians(70),
                          math.radians(-33),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    mid3 = [math.radians(74),
                          math.radians(7),
                          math.radians(-47),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]




    pluck1 = [math.radians(90),
                math.radians(55),
                math.radians(-51),
                math.radians(1),
                math.radians(-5),
                math.radians(1)]
    pluck2 = [math.radians(70),
                math.radians(41),
                math.radians(-41),
                math.radians(-15),
                math.radians(-0),
                math.radians(0)]
    pluck3 = [math.radians(72),
                math.radians(72),
                math.radians(-114),
                math.radians(5),
                math.radians(0),
                math.radians(0)]
    dropr = [math.radians(-31),
                math.radians(47),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]
    dropl = [math.radians(11),
                math.radians(47),
                math.radians(0),
                math.radians(0),
                math.radians(0),
                math.radians(0)]
    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose(1,"open")
        rospy.sleep(1)
        ur5.set_joint_angles(2,mid1)
        rospy.sleep(1)
        ur5.set_joint_angles(2,pluck1)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"close")
        rospy.sleep(1)
        ur5.set_joint_angles(2,mid1)
        rospy.sleep(1)
        ur5.set_joint_angles(2,dropr)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"open")
        rospy.sleep(1)

        ur5.set_joint_angles(2,mid2)
        rospy.sleep(1)
        ur5.set_joint_angles(2,pluck2)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"close")
        rospy.sleep(1)
        ur5.set_joint_angles(2,dropl)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"open")
        rospy.sleep(1)

        ur5.set_joint_angles(2,mid3)
        rospy.sleep(1)
        ur5.set_joint_angles(2,pluck3)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"close")
        rospy.sleep(1)
        ur5.set_joint_angles(2,mid3)
        rospy.sleep(1)
        ur5.set_joint_angles(2,dropr)
        rospy.sleep(1)
        ur5.go_to_predefined_pose(1,"open")
        rospy.sleep(1)

        

    del ur5


if __name__ == '__main__':
    main()
