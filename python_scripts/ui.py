#!/usr/bin/env python

from moveit_ur5 import MoveGroupPythonInteface
import threadingimport sys
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from Tkinter import *
import tkMessageBox
import rospy
import copy
import math


class MES:
    def __init__(self):
        False
        #self.robot = MoveGroupPythonInteface()


    def go_to_cords(self):
        try:
            values = [int(x.get()) for x in self.display]
        except Exception:
            tkMessageBox.showerror('ERROR', 'Ongeldige coordinaten')
            return
        x, y, z = values
        self.robot.go_to_pose_goal(x, y, z)


    def window(self):
        root = Tk()
        root.title('PCB Picker')


        # HUIDIGE LOCATIE ----------------------------------------------------
        Label(root, text='Huidige positie: ' + str((1,2,2))).grid(
            columnspan=2, sticky=W)

        # USER INPUT ---------------------------------------------------------
        Label(root, text='Geef hier een input').grid(row=1, columnspan=2)
        Label(root, text='x').grid(row=2, column=0)
        Label(root, text='y').grid(row=3, column=0)
        Label(root, text='z').grid(row=4, column=0)
        # Input boxes
        self.display = [Entry(root) for i in range(3)]
        [self.display[i].grid(row=i+1, column=1, sticky=W+E) for i in range(3)]
        Button(root, text='Execute', command=self.go_to_cords).grid(
            row=4, columnspan=2, sticky=W+E)

        # TAKEN OVERZICHT ----------------------------------------------------



        root.mainloop()


def main():
    app = MES()
    app.window()


if __name__ == '__main__':
    main()