#!/usr/bin/env python

from moveit_ur5 import MoveGroupPythonInteface
import threading
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from Tkinter import *
import tkMessageBox
import rospy
import copy
import math
import sys


class MES:
    def __init__(self):
        False

        with open('./taken.txt', 'r') as file:
            self.inhoud = file.read()
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
        Label(root, text='x').grid(row=1, column=0)
        Label(root, text='y').grid(row=2, column=0)
        Label(root, text='z').grid(row=3, column=0)
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


def main():
    app = MES()
    app.window()


if __name__ == '__main__':
    main()