#!/usr/bin/env python

'''
NOTES! 
Voor dit bestand uitgevoerd kan worden moet er eest in een ander terminal hetvolgende uitgevoerd worden:
roslaunch ur5_moveit_config demo.launch
    Dit bestand is opgeslagen in 
    /opt/ros/kinetic/share



/ur5_moveit_config/config/ur5.srdf



Om ur5 te openen in gazebo gooi deze commands in 2 terminals (en open vervolgens dit script in een 3e): 
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true



'''
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi



class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher

        # PRODUCT PARAMETERS----------
        self.product_locations = [
                            [.5, .5, .5], 
                            [.4, .4, .4], 
                            [.3, .3, .3],
                        ]
        self.place_locations = [
                            [.5, .5, .4],
                            [.5, .5, .4],
                            [.5, .5, .4],
                        ]
        assert len(self.place_locations) == len(self.product_locations)
        self.productcount = len(self.product_locations)

    def go_to_pose_goal(self, x, y, z, w=0):
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling stop() ensures that there is no residual movement
        group.stop()
        group.clear_pose_targets()

        # # Kunnen weg?
        current_pose = self.group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)

    def get_pose():
        return self.group.get_current_pose().pose

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