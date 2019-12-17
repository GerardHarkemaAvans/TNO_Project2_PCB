#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi



class MoveGroupPythonInteface(object):
    def __init__(self, robot_name):
        super(MoveGroupPythonInteface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_name = robot_name
        time.sleep(.5)
        if robot_name == 'ur5':
            group_name = "manipulator"
        elif robot_name == 'panda':
            group_name = 'panda_arm'
        # else:
        #     print(robot_name + ' not regocnized')
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        # PRODUCT PARAMETERS----------
        ## All the inner lists must have 7 elements
        if robot_name == 'ur5':
            self.product_app_loc = [
                                [.38, .5437, .3622, -1.57, 1.57, 0],
                                [.601, .21, -.005, 1.57, 0, 1.52],
                                [.693, -.03, .3, 1.5, 1.05, -1.65],
            ]
            self.product_locations = [
                                [.38, .5435, .165, -1.57, 1.57, 0], 
                                [.601, .237, -.005, 1.57, 0, 1.52],
                                [.693, -.133, .167, 1.5, 1.05, -1.65],
                            ] 
            self.product_leave_loc1 = [
                                [.39, .53, .175, -1.57, 1.57, 0],
                                [.601, .237, .02, 1.57, 0, 1.52],
                                [.693, -.03, .3, 1.5, 1.05, -1.65],
            ]
            self.product_leave_loc2 = [
                                [.39, .5, .5, -1.57, 1.57, 0],
                                [.601, .237, .3, 1.57, 0, 1.52],
                                [],
            ]
            self.place_locations = [
                                [.673, -.008, .165, 0.01, 1.59, 0],
                                [.675, -.012, .177, 3.14, 1.61, -.03],
                                [.674, -.01, .2, .01, 1.59, 0],
                            ]
        elif robot_name == 'panda':
            self.product_app_loc = [
                                [.3678, .5182, .3, 3.14, 0, 0.78],
                                [.5743, .2982, .07, -1.57, -0.78, 0],
                                [.6916, -.0132, .3, 2.356, 0.6, 0.6],
            ]
            self.product_locations = [
                                [.3678, .5182, .15, 3.14, 0, 0.78], 
                                [.5743, .3982, .07, -1.57, -0.78, 0],
                                [.6916, -.1132, .152, 2.356, 0.6, 0.6],
                            ]
            self.product_leave_loc = [
                                [.3678, .5182, .3, 3.14, 0, 0.78],
                                [.5743, .3982, .2, -1.57, -0.78, 0],
                                [.6916, -.0132, .3, 2.356, 0.6, 0.6],
            ]
            self.place_locations = [
                                [.66, 0, .13, 3.14, 0, 2.356],
                                [.66, 0, .162, 3.14, 0, 2.356],
                                [.66, 0, .194, 3.14, 0, 2.356],
                            ]

    def go_to_pose_goal(self, x, y, z, rx=pi, ry=0, rz=0, w=None):
        group = self.group

        coords = [x, y, z, rx, ry, rz]
        if w != None:
            coords.append(w)
        group.set_pose_target(coords)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling stop() ensures that there is no residual movement
        group.stop()
        group.clear_pose_targets()

    def get_pose():
        return self.group.get_current_pose().pose

    def go_to_joint_state(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[1] = -.977
        self.group.go(joint_goal, wait=True)

        if self.robot_name == 'ur5':
            pass
            ## todo dit nog werkend krijgen - lijkt erop alsof de box niet goed toegevoegd is
            # p = moveit_commander.PoseStamped()
            # p.header.frame_id = self.robot.get_planning_frame()
            # p.pose.position.x = 0
            # p.pose.position.y = 0
            # p.pose.position.z = 1
            # self.scene.add_box('Ground', p, (2,2,.15))
            # print('Box added')

        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = .63
        joint_goal[2] = .609
        self.group.go(joint_goal, wait=True)

        # joint_goal = self.group.get_current_joint_values()
        # joint_goal[3] = 5.07
        # joint_goal[4] = 4.712
        # joint_goal[5] = -2.511
        # self.group.go(joint_goal, wait=True)

        self.group.stop()


def temp():
    robot = MoveGroupPythonInteface()
    while True:
        try:
            x = float(raw_input('x: '))
            y = float(raw_input('y: '))
            z = float(raw_input('z: '))
            robot.go_to_pose_goal(x, y, z)
        except Exception:
            print('ongeldige input, probeer opnieuw')


def main():
    robot = MoveGroupPythonInteface()
    robot.go_to_pose_goal(.5, .5, .5)

if __name__ == '__main__':
    temp()
