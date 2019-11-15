#!/usr/bin/env python

from moveit_ur5 import MoveGroupPythonInteface
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from Tkinter import *
import tkMessageBox
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



'''


class MES:
    def __init__(self, robot_name):
        with open('./taken.txt', 'r') as file:
            self.inhoud = file.read()

        self.robot = MoveGroupPythonInteface(robot_name)

    def go_to_cords(self):
        try:
            values = [float(x.get().replace('pi', math.pi)) for x in self.display]
        except Exception:
            tkMessageBox.showerror('ERROR', 'Ongeldige coordinaten')
            return
        x, y, z, w = values
        self.robot.go_to_pose_goal(x, y, z, w)

    def execute_all(self):
        for taak in self.inhoud.split('\n'):
            if taak.startswith('go_product'):
                # We have to to -1 bc python starts counting at 0
                idx = int(taak.split('-')[1]) - 1 
                product_location = self.robot.product_locations[idx]
                x, y, z = product_location
                self.robot.go_to_pose_goal(x, y, z)
            
            elif taak.startswith('go_up'):
                self.robot.go_to_pose_goal(x, y, 0.5)  # todo mogelijk z + 0.5 van maken
            
            elif taak.startswith('go_placeloc'):
                idx = int(taak.split('-')[1]) - 1 
                place_location = self.robot.place_locations[idx]
                x, y, z = place_location
                self.robot.go_to_pose_goal(x, y, z)
            
            else:
                tkMessageBox.showerror('ERROR', 'Devolgende regel is niet '
                    'herkend\n' + taak)

    def get_pos(self):
        pos = self.robot.group.get_current_pose()
        tkMessageBox.showinfo('Position', str(pos.pose))

        pass

    def window(self):
        root = Tk()
        root.title('PCB Picker')

        # HUIDIGE LOCATIE ----------------------------------------------------
        Button(root, text='Get pos', command=self.get_pos).grid(sticky=W)
        self.pos = StringVar(root)
        Label(root, textvariable=self.pos).grid(column=1, sticky=W)
        # Label(root, text='Huidige positie: ' + str((1,2,2))).grid(column=1,
        #     sticky=W)

        # USER INPUT ---------------------------------------------------------
        Label(root, text='Geef hier een input').grid(row=1, columnspan=2, 
            sticky=W)
        Label(root, text='x (groen)').grid(row=2, column=0, sticky=W)
        Label(root, text='y (rood)').grid(row=3, column=0, sticky=W)
        Label(root, text='z (blauw)').grid(row=4, column=0, sticky=W)
        Label(root, text='w').grid(row=5, column=0, sticky=W)
        # Input boxes
        self.display = [Entry(root) for i in range(4)]
        [self.display[i].grid(row=i+2, column=1, sticky=W+E) for i in range(4)]
        Button(root, text='Go To', command=self.go_to_cords).grid(
            row=6, columnspan=2, sticky=W+E)

        # TAKEN OVERZICHT ----------------------------------------------------
        taken = self.inhoud.split('\n')
        for idx, taak in enumerate(taken):
            Label(root, bg='red', text=taak.replace('_', ' ').replace(
                '-', ' ')).grid(row=7+idx, columnspan=2, sticky=W)
        Button(root, text='Execute All Tasks', command=self.execute_all).grid(
            row=8+idx, columnspan=2, sticky=W+E)

        root.mainloop()



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
        command = 'roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true'
    process = subprocess.call(command.split(' '))
    process.communicate()


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
    robot_name = raw_input("[ur5] or [panda]: ")
    if robot_name not in ['ur5', 'panda']:
        raise NameError('robot not regocnized')
    if robot_name == 'ur5':
        ur5 = True
    if robot_name == 'panda':
        panda = True

    resp = raw_input("open 'roslaunch files? y/[n]: ")
    if resp.lower() in ['y', 'j', 'yes', 'ja']:
        roslaunch_thread()

    app = MES(robot_name)
    app.window()


if __name__ == '__main__':
    main()