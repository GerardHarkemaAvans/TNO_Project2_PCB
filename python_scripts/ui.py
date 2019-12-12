#!/usr/bin/env python

from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from moveit_ur5 import MoveGroupPythonInteface
import geometry_msgs.msg
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import tkMessageBox
import numpy as np
import subprocess
import threading
import random
import roslib
import rospy
import copy
import math
import time
import sys

# Notes van x, y, z, w naar rx, ry, rz:
# from geometry_msgs.msg import Quaternion
# rot = Quaternion(x, y, z, w)
# rot.deserialize


from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
roslib.load_manifest('robotiq_2f_gripper_control')
# from Robotiq2FGripperRtuNode import mainLoop


GRIPPERPORT = '/dev/ttyUSB0'
MAINLOOPRUNNING = False


# Make this file compeditable on both python2 and python3
if sys.version_info[0] == 3:
    raw_input = input
    from tkinter import *
else:
    from Tkinter import *


class MES:
    def __init__(self, robot_name):
        with open('./taken.txt', 'r') as file:
            self.inhoud = file.read()
        self.robot_name = robot_name
        self.robot = MoveGroupPythonInteface(self.robot_name)
        self.rx = np.pi
        if self.robot_name == 'panda':
            self.ry = 0
        elif self.robot_name == 'ur5':
            self.ry = np.pi / 2
        self.rz = 0
        
        # ADDING TABLE -------------------------------------------------------
        p = moveit_commander.PoseStamped()
        p.header.frame_id = self.robot.robot.get_planning_frame()
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = -.05  # correctie dat tafel lager staat
        self.robot.scene.add_box('table', p, (1.25, 1.25, .01))
        time.sleep(.2)

        #ADDING BACK WALL ----------------------------------------------------
        p = moveit_commander.PoseStamped()
        p.header.frame_id = self.robot.robot.get_planning_frame()
        p.pose.position.x = -.25
        p.pose.position.y = 0
        p.pose.position.z = 0
        self.robot.scene.add_box('backwall', p, (.01, 2, 2))
        time.sleep(.2)

        #ADDING BACK HOLDER --------------------------------------------------
        p = moveit_commander.PoseStamped()
        p.header.frame_id = self.robot.robot.get_planning_frame()
        p.pose.position.x = .66
        p.pose.position.y = -.4
        p.pose.position.z = .2
        self.robot.scene.add_box('backholder1', p, (.2, .2, .1))
        time.sleep(.5)

        #ADDING OBJECT1 ---------------------------------------------------
        p = moveit_commander.PoseStamped()
        p.header.frame_id = self.robot.robot.get_planning_frame()
        p.pose.position.x = 0.5
        p.pose.position.y = 0.5
        p.pose.position.z = 0.01
        self.robot.scene.add_box('OBJECT1', p, (0.05, 0.05, 0.02))
        time.sleep(.5)

        self.finger_pub = rospy.Publisher(
            '/move_group/fake_controller_joint_states',
            sensor_msgs.msg.JointState,
            queue_size=20)

        # if panda and not real:
        #     time.sleep(1)
        #     print('OPENING GRIPPER')
        #     self.control_gripper(100)
        #     time.sleep(1.5)
        

    def go_to_cords(self):
        try:
            values = [float(x.get()) for x in self.display]
        except Exception:
            tkMessageBox.showerror('ERROR', 'Ongeldige coordinaten')
            return
        x, y, z, rx, ry, rz = values
        print('Received ', values)
        self.robot.go_to_pose_goal(x, y, z, rx, ry, rz)

    def go_up(self):
        x = self.robot.group.get_current_pose().pose.position.x
        y = self.robot.group.get_current_pose().pose.position.y
        z = self.robot.group.get_current_pose().pose.position.z
        if np.round(z, 3) != 0.3:
            self.robot.go_to_pose_goal(x, y, 0.3, self.rx, self.ry, 
                self.rz)

    def execute_all(self, gripper=False):
        for i, taak in enumerate(self.inhoud.split('\n')):
            Label(self.root, bg='red', text=taak.replace('_', ' ').replace(
                '-', ' ')).grid(row=9+i, columnspan=2, sticky=W+E)
        if gripper:
            self.control_gripper(100)  # always open gripper b4 starting
            time.sleep(1.2)

        for i, taak in enumerate(self.inhoud.split('\n')):
            x = self.robot.group.get_current_pose().pose.position.x
            y = self.robot.group.get_current_pose().pose.position.y
            z = self.robot.group.get_current_pose().pose.position.z

            # We have to to -1 bc python starts counting at 0
            # Read product location

            if taak.startswith('go_product'):
                # Update label
                Label(self.root, bg='orange', text=taak.replace('_', ' ').replace(
                    '-', ' ')).grid(row=9+i, columnspan=2, sticky=W+E)
                time.sleep(0.03)

                # Read product approach location details
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.product_app_loc[idx]
                if len(product_location) == 3:
                    gx, gy, gz = product_location
                    grx, gry, grz = 3.14, 1.57, 0  # todo aanpassen voor panda
                else:
                    gx, gy, gz, grx, gry, grz = product_location
                self.robot.go_to_pose_goal(gx, gy, gz, grx, gry, grz)

                # Read product location details
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.product_locations[idx]
                if len(product_location) == 3:
                    gx, gy, gz = product_location
                    grx, gry, grz = 3.14, 1.57, 0  # todo aanpassen voor panda
                else:
                    gx, gy, gz, grx, gry, grz = product_location
                self.robot.go_to_pose_goal(gx, gy, gz, grx, gry, grz)
                if gripper:
                    self.control_gripper(0)
                    time.sleep(1.2)

                # Read product leave location details
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.product_leave_loc[idx]
                if len(product_location) == 3:
                    gx, gy, gz = product_location
                    grx, gry, grz = 3.14, 1.57, 0  # todo aanpassen voor panda
                else:
                    gx, gy, gz, grx, gry, grz = product_location
                self.robot.go_to_pose_goal(gx, gy, gz, grx, gry, grz)

            elif taak.startswith('go_placeloc'):
                # Update label
                Label(self.root, bg='orange', text=taak.replace('_', ' ').replace(
                    '-', ' ')).grid(row=9+i, columnspan=2, sticky=W+E)
                time.sleep(0.03)

                # Read product location details
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.place_locations[idx]
                if len(product_location) == 3:
                    gx, gy, gz = product_location
                    grx, gry, grz = 3.14, 1.57, 0  # todo aanpassen voor panda
                else:
                    gx, gy, gz, grx, gry, grz = product_location

                # First go above the product
                self.robot.go_to_pose_goal(gx, gy, 0.3, grx, gry, grz)
                # And go down now
                self.robot.go_to_pose_goal(gx, gy, gz, grx, gry, grz)
                if gripper:
                    self.control_gripper(100)
                    time.sleep(1.2)
                # And go up again
                self.robot.go_to_pose_goal(gx, gy, 0.3, grx, gry, grz)

            else:
                tkMessageBox.showerror('ERROR', 'Devolgende regel is niet '
                    'herkend\n' + taak)
            print(i, taak, gx, gy, gz, grx, gry, grz)  # Debug
            Label(self.root, bg='green', text=taak.replace('_', ' ').replace(
                '-', ' '), fg='white').grid(row=9+i, columnspan=2, sticky=W+E)
        


    def execute_all_thread(self):
        thread = threading.Thread(target=self.execute_all)
        thread.start()

    def get_pos(self):
        pos = self.robot.group.get_current_pose()
        print(str(pos.pose))
        tkMessageBox.showinfo('Position', str(pos.pose))
        pos2 = self.robot.group.get_current_joint_values()
        print('Joint values:')
        print(str(pos2))
        tkMessageBox.showinfo('Joint Values', str(pos2))

    def add_objects(self):
        x, y, z, dx, dy, dz = [x.get() for x in self.boxcoords]
        p = moveit_commander.PoseStamped()
        p.header.frame_id = self.robot.robot.get_planning_frame()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        self.robot.scene.add_box(self.boxname.get(), p, (float(dx), float(dy), float(dz)))
        time.sleep(.5)
        print('Toegevoegd')

    def window(self):
        self.root = Tk()
        self.root.title('PCB Picker')

        # HUIDIGE LOCATIE ----------------------------------------------------
        Button(self.root, text='Go Home', command=self.robot.go_to_joint_state).grid(
            sticky=W+E)
        Button(self.root, text='Get pos', command=self.get_pos).grid(
            sticky=W, row=0, column=1)
        self.pos = StringVar(self.root)
        Label(self.root, textvariable=self.pos).grid(column=1, sticky=W)

        # USER INPUT ---------------------------------------------------------
        Label(self.root, text='Geef hier een input').grid(row=1, columnspan=2, 
            sticky=W)
        for idx, elem in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz']):
            # x=groen, y=rood, z=blauw
            Label(self.root, text=elem).grid(row=2+idx, column=0, sticky=W)

        # Input boxes --------------------------------------------------------
        self.display = [Entry(self.root) for i in range(6)]
        [self.display[i].grid(row=i+2, column=1, sticky=W+E) for i in range(6)]
        Button(self.root, text='Go To', command=self.go_to_cords).grid(
            row=8, columnspan=2, sticky=W+E)

        # TAKEN OVERZICHT ----------------------------------------------------
        taken = self.inhoud.split('\n')
        for idx, taak in enumerate(taken):
            Label(self.root, bg='red', text=taak.replace('_', ' ').replace(
                '-', ' ')).grid(row=9+idx, columnspan=2, sticky=W+E)
        Button(self.root, text='Execute All Tasks', command=
            self.execute_all_thread).grid(row=10+idx, columnspan=2, sticky=W+E)

        # GRIPPER BUTTONS ----------------------------------------------------
        Label(self.root, text='Gripper control').grid(row=11+idx, sticky=W,
            columnspan=2)
        Button(self.root, text='open', command=lambda: self.control_gripper(100)
            ).grid(row=12+idx, sticky=W+E, column=0)
        Button(self.root, text='close', command=lambda: self.control_gripper(0)
            ).grid(row=12+idx, sticky=W, column=1)

        # BOXEN TOEVOEGEN ----------------------------------------------------
        Label(self.root, text='Box toevoegen').grid(row=13+idx, sticky=W, 
            columnspan=2)
        for idx2, elem in enumerate(['x', 'y', 'z', 'dx', 'dy', 'dz']):
            Label(self.root, text=elem).grid(row=14+idx+idx2, sticky=W)
        self.boxcoords = [Entry(self.root) for i in range(6)]
        [self.boxcoords[idx2].grid(row=14+idx+idx2, column=1) for idx2 in range(6)]
        Label(self.root, text='naam').grid(row=15+idx+idx2, column=0, sticky=W)
        self.boxname = Entry(self.root)
        self.boxname.grid(row=15+idx+idx2, column=1)
        Button(self.root, text='Toevoegen', command=self.add_objects).grid(
            row=17+idx+idx2, columnspan=2, sticky=W+E)

        self.root.mainloop()


    def control_gripper(self, action):
        global MAINLOOPRUNNING, pub

        try:
            action = int(action)
        except Exception:
            raise TypeError('Must take an int as arg')
        if action > 100 or action < 0:
            raise ValueError('input too big or too small')
        action_to_255 = 255 - int(action * 2.55)

        if real and ur5:
            if not MAINLOOPRUNNING:
                MAINLOOPRUNNING = True
                activate_gripper_thread()
                pub = rospy.Publisher('Robotiq2FGripperRobotOutput', 
                    outputMsg.Robotiq2FGripper_robot_output)

                command = outputMsg.Robotiq2FGripper_robot_output()
                command.rACT = 1
                command.rGTO = 1
                command.rSP  = 255
                command.rFR  = 150
                pub.publish(command)
                time.sleep(1)

            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rATR = 0
            command.rPR = action_to_255
            command.rSP = 255
            command.rFR = 25
            pub.publish(command)
        
        if panda:
            value = action / 100 * 0.04
            print(value)
            jointstate = sensor_msgs.msg.JointState()
            jointstate.name += ['panda_finger_joint1', 'panda_joint1', 
            'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 
            'panda_joint6', 'panda_joint7']
            jointstate.position += [value] + self.robot.group.get_current_joint_values()
            self.finger_pub.publish(jointstate)


def run_roslaunch1():
    if panda:
        command = 'roslaunch panda_moveit_config demo.launch'
    if ur5:
        command = 'roslaunch ur_gazebo ur5.launch'
    process = subprocess.Popen(command.split(' '))
    process.communicate()


def run_roslaunch2():
    if panda:
        command = 'rosrun moveit_commander moveit_commander_cmdline.py'
    if ur5:
        command = 'roslaunch ur5_moveit_config ' +\
        'ur5_moveit_planning_execution.launch sim:=true limited:=true'
    process = subprocess.call(command.split(' '))
    try:
        process.communicate()
    except Exception:
        sys.exit(0)


def roslaunch_thread():
    thread1 = threading.Thread(target=run_roslaunch1)
    thread1.start()
    time.sleep(6)

    thread2 = threading.Thread(target=run_roslaunch2)
    thread2.start()
    time.sleep(5)


def activate_gripper():
    command = 'python Robotiq2FGripperRtuNode.py ' + GRIPPERPORT
    process = subprocess.Popen(command.split(' '))
    process.communicate()
    time.sleep(1)

    command2 = 'python init_gripper_node'
    process = subprocess.Popen(command2.split(' '))
    process.communicate()
    time.sleep(1)


def activate_gripper_thread():
    thread = threading.Thread(target=activate_gripper)
    thread.start()
    time.sleep(4)



def main():
    global ur5, panda, real
    ur5, panda, real = False, False, False
    robot_name = raw_input("[ur5] or [panda]: ")

    ## For debugging
    # robot_name = 'ur5'
    # robot_name = 'panda'

    resp = raw_input('simulation or real hardware? s/[r] ')
    if resp == 's':
        real = False
    elif resp == 'r':
        real = True
    else:
        print('Bad input')
        return

    if robot_name.lower() in ['ur5', 'u']:
        robot_name = 'ur5'
        ur5 = True
    elif robot_name.lower() in ['panda', 'p']:
        robot_name = 'panda'
        panda = True
    else:
        print('Bad input')
        return

    resp = 'y'
    resp = raw_input("open 'roslaunch files? Y/[N]: ")
    if resp.lower() in ['y', 'j', 'yes', 'ja']:
        roslaunch_thread()
    elif resp.lower() not in ['n', 'no', 'ne', 'nee']:
        print('Bad input')
        return

    app = MES(robot_name)
    app.window()

if __name__ == '__main__':
    main()
