#!/usr/bin/env python


"""@package docstring
ROS node for controling a Robotiq 2F gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseRobotiq2FGripper object and adds a comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command. The gripper status is published on the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type. The node subscribes to the 'Robotiq2FGripperRobotOutput' topic for new commands using the 'Robotiq2FGripper_robot_output' msg type. Examples are provided to control the gripper (Robotiq2FGripperSimpleController.py) and interpreting its status (Robotiq2FGripperStatusListener.py).
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

def mainLoop(device):
    
    #Gripper is a 2F with a TCP connection
    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    #We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node('robotiq2FGripper')

    #The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=10)

    temp = inputMsg.Robotiq2FGripper_robot_input

    #The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
    rospy.Subscriber('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, gripper.refreshCommand)
    

    #We loop
    while not rospy.is_shutdown():

      #Get and publish the Gripper status
      status = gripper.getStatus()
      pub.publish(status)     

      #Send the most recent command
      gripper.sendCommand()

      #Wait a little
      rospy.sleep(0.05)
            
if __name__ == '__main__':
    try:
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException: 
      pass
