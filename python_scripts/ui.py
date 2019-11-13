#!/usr/bin/env python

from moveit_ur5 import MoveGroupPythonInteface
import threading
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


class MES:
    def __init__(self):
        False

        with open('./taken.txt', 'r') as file:
            self.inhoud = file.read()
        raw_input("PRESS ENTER TO CONTINUE") # temp
        self.robot = MoveGroupPythonInteface()

    def go_to_cords(self):
        try:
            values = [float(x.get()) for x in self.display]
        except Exception:
            tkMessageBox.showerror('ERROR', 'Ongeldige coordinaten')
            return
        x, y, z = values
        self.robot.go_to_pose_goal(x, y, z)

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

    def window(self):
        root = Tk()
        root.title('PCB Picker')

        # HUIDIGE LOCATIE ----------------------------------------------------
        Label(root, text='Huidige positie: ' + str((1,2,2))).grid(
            columnspan=2, sticky=W)

        # USER INPUT ---------------------------------------------------------
        Label(root, text='Geef hier een input').grid(row=1, columnspan=2)
        Label(root, text='x (groen)').grid(row=1, column=0, sticky=W)
        Label(root, text='y (rood)').grid(row=2, column=0, sticky=W)
        Label(root, text='z (blauw)').grid(row=3, column=0, sticky=W)
        # Input boxes
        self.display = [Entry(root) for i in range(3)]
        [self.display[i].grid(row=i+1, column=1, sticky=W+E) for i in range(3)]
        Button(root, text='Go To', command=self.go_to_cords).grid(
            row=4, columnspan=2, sticky=W+E)

        # TAKEN OVERZICHT ----------------------------------------------------
        taken = self.inhoud.split('\n')
        for idx, taak in enumerate(taken):
            Label(root, bg='red', text=taak.replace('_', ' ').replace(
                '-', ' ')).grid(row=5+idx, columnspan=2, sticky=W)
        Button(root, text='Execute All Tasks', command=self.execute_all).grid(
            row=6+idx, columnspan=2, sticky=W+E)

        root.mainloop()



def run_roslaunch1():
    process = subprocess.Popen('roslaunch ur_gazebo ur5.launch'.split(' '))
    process.communicate()


def run_roslaunch2():
    process = subprocess.call(
        'roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true'.split(' '))
    process.communicate()


def roslaunch_thread():
    thread1 = threading.Thread(target=run_roslaunch1)
    thread1.start()
    time.sleep(5)

    thread2 = threading.Thread(target=run_roslaunch2)
    thread2.start()
    time.sleep(5)


def main():
    resp = raw_input("open 'roslaunch ur_gazebo ur5.launch'? ")
    if resp.lower() in ['y', 'j', 'yes', 'ja']:
        roslaunch_thread()

    app = MES()
    app.window()


if __name__ == '__main__':
    main()