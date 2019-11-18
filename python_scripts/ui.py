#!/usr/bin/env python

from moveit_ur5 import MoveGroupPythonInteface
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from Tkinter import *
import tkMessageBox
import numpy as np
import subprocess
import threading
import rospy
import copy
import math
import time
import sys


'''
Packages voor franka:
    franka_gazebo
    libfranka
    panda_moveit_config
    panda_simulation


To start panda simulation in rviz:
roslaunch panda_moveit_config demo.launch
'''


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

    def go_to_cords(self):
        try:
            values = [float(x.get()) for x in self.display]
        except Exception:
            tkMessageBox.showerror('ERROR', 'Ongeldige coordinaten')
            return
        x, y, z, rx, ry, rz = values
        print('Received ', values)
        self.robot.go_to_pose_goal(x, y, z, rx, ry, rz)

    def go_home_position(self):
        self.robot.go_to_home()

    def execute_all(self):
        for i, taak in enumerate(self.inhoud.split('\n')):
            x = self.robot.group.get_current_pose().pose.position.x
            y = self.robot.group.get_current_pose().pose.position.y
            z = self.robot.group.get_current_pose().pose.position.z
            if taak.startswith('go_product'):
                if np.round(x, 1) == 0.8 and np.round(y, 1) == 0.2 and\
                        np.round(z, 1) == 0.0:
                    print('NO GOOD!!')
                    break
                    # temp
                else:
                    print('NIET IN UR5 BEGIN POSITIE')

                Label(self.root, bg='orange', text=taak.replace('_', ' ').replace(
                    '-', ' ')).grid(row=7+i, columnspan=2, sticky=W+E)
                time.sleep(0.01)
                # START BY GOING TO 50 CM HEIGHT -----------------------------
                if np.round(z, 3) != 0.5:
                    self.robot.go_to_pose_goal(x, y, 0.5, self.rx, self.ry, 
                        self.rz)

                # We have to to -1 bc python starts counting at 0
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.product_locations[idx]
                x_place, y_place, z_place = product_location
                # First go above the product
                self.robot.go_to_pose_goal(x_place, y_place, 0.5, self.rx, 
                    self.ry, self.rz)
                # And go down now
                self.robot.go_to_pose_goal(x_place, y_place, z_place, self.rx, 
                    self.ry, self.rz)
            
            
            elif taak.startswith('go_placeloc'):
                Label(self.root, bg='orange', text=taak.replace('_', ' ').replace(
                    '-', ' ')).grid(row=7+i, columnspan=2, sticky=W+E)
                time.sleep(0.02)
                if np.round(z, 3) != 0.5:
                    self.robot.go_to_pose_goal(x, y, 0.5, self.rx, self.ry, 
                        self.rz)

                idx = int(taak.split('-')[1]) - 1 
                place_location = self.robot.place_locations[idx]
                x_place, y_place, z_place = place_location
                self.robot.go_to_pose_goal(x_place, y_place, 0.5, self.rx, 
                    self.ry, self.rz)
                self.robot.go_to_pose_goal(x_place, y_place, z_place, self.rx, 
                    self.ry, self.rz)
            
            else:
                tkMessageBox.showerror('ERROR', 'Devolgende regel is niet '
                    'herkend\n' + taak)
            Label(self.root, bg='green', text=taak.replace('_', ' ').replace(
                '-', ' '), fg='white').grid(row=7+i, columnspan=2, sticky=W+E)
        # End by going up
        x = self.robot.group.get_current_pose().pose.position.x
        y = self.robot.group.get_current_pose().pose.position.y
        z = self.robot.group.get_current_pose().pose.position.z
        if np.round(z, 3) != 0.5:
            self.robot.go_to_pose_goal(x, y, 0.5)


    def execute_all_thread(self):
        thread = threading.Thread(target=self.execute_all)
        thread.start()

    def get_pos(self):
        pos = self.robot.group.get_current_pose()
        tkMessageBox.showinfo('Position', str(pos.pose))

    def window(self):
        self.root = Tk()
        self.root.title('PCB Picker')

        # HUIDIGE LOCATIE ----------------------------------------------------
        Button(self.root, text='Go Home', command=self.go_home_position).grid(sticky=W+E)
        Button(self.root, text='Get pos', command=self.get_pos).grid(
            sticky=W, row=0, column=1)
        self.pos = StringVar(self.root)
        Label(self.root, textvariable=self.pos).grid(column=1, sticky=W)
        # Label(root, text='Huidige positie: ' + str((1,2,2))).grid(column=1,
        #     sticky=W)

        # USER INPUT ---------------------------------------------------------
        Label(self.root, text='Geef hier een input').grid(row=1, columnspan=2, 
            sticky=W)
        for idx, elem in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz']):
            # x=groen, y=rood, z=blauw
            Label(self.root, text=elem).grid(row=2+idx, column=0, sticky=W)

        # Input boxes
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
        self.root.lift()
        self.root.mainloop()


def run_roslaunch1():
    if panda:
        # Dit command is voor het plannen -- dus moet aangepast worden
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


def main():
    global ur5, panda
    ur5, panda = False, False
    robot_name = 'ur5'  # for faster startup
    robot_name = raw_input("[ur5] or [panda]: ")

    ## For debugging
    # robot_name = 'ur5'
    # robot_name = 'panda'

    if robot_name not in ['ur5', 'panda']:
        raise NameError('robot not regocnized')
    if robot_name == 'ur5':
        ur5 = True
    if robot_name == 'panda':
        panda = True

    resp = 'j'
    resp = raw_input("open 'roslaunch files? y/[n]: ")
    if resp.lower() in ['y', 'j', 'yes', 'ja']:
        roslaunch_thread()

    app = MES(robot_name)
    app.window()


if __name__ == '__main__':
    main()