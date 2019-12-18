# Project Pick & Place voor TNO
Dit is een project voor de minor Vision & Robotics 2019-2020 te Avans Breda. In dit project wordt er met een cobot gewerkt. Een cobot is een robot die bedoelt is om samen te werken met mensen in een gemeenschappelijke omgeving. Centraal bij dit project staat dat de software modulair moet zijn. Dat wilt zeggen dat de software op verschillende robots moet werken (onder andere ur5 & panda). Ook moet de robot verschillende halffabricaten op kunnen pakken. Het project is gerealiseert in ROS kinetic.


# Resultaten
[youtu.be/bivQM2FoDJs](https://youtu.be/bivQM2FoDJs)


# Installeer dit project:
Installeer de volgende afhankelijkheden:
```bash
sudo apt install ros-kinetic-full
sudo apt install ros-kinetic-moveit
sudo apt install ros-kinetic-ur-description
sudo apt install ros-kinetic-franka-description
```

Maak vervolgens een catkin workspace aan en ga naar de src. Voer daarna de commands hier beneden uit:
```bash
cd ~/catkin_ws/src

git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
git clone https://github.com/NielsBos1996/TNO_Project2_PCB
```

Vervolgens moeten er enkele python packages geinstalleerd worden:
```bash
pip install pyyalm
pip install rospkg
```

En build:
```bash
# Installeer dependencies
rosdep install --from-paths . --ignore-src --rosdistro kinetic
catkin_make
```

Verplaats het bestand .../TNO_Project2_PCB/config/controllers.yaml naar .../ur5_moveit_config/config/controllers.yaml. Gebruik het command `roscd ur5_moveit_config` om erachter te komen waar moveit_config geinstalleerd is.
Open het bestand .../ur5_moveit_config/config/controllers.yaml en vervang de 3e regel van het bestand voor `action_ns: scaled_pos_traj_controller/follow_joint_trajectory`.


# Verbinden met de ur5
Stel de DNS van de ur5 in op `192.168.1.30`. Kijk vervolgens wat de naam is van de ehternetadapter op de pc met het command `ifconfig` (vaak lijkt het op `enp4s0f2`). Stop de ethernet kabel van de ur5 en de usb kabel van de gripper in de pc. 
```bash
# Stel nu in dat de pc gaat luisteren naar packets op het kanaal `192.168.1.30`
sudo ifconfig `ethernetnaam` 192.168.1.30 netmask 255.255.255.0

# En verbind met de ur5
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.102

# Start nu het bestand `urcapexternalcontrol.urp` op de ur5
# Start planning launch file
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true

# Nu kan de interface gestart worden
cd ~/catkin_ws/src/TNO_Project2_PCB/python_scripts/
python ui.py

# OPTIONEEL start rviz om de echte hardware in een simulatie te hebben
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```


# Verbinden met de gripper
Stop de usb van de gripper in de pc. Controlleer in welke port de gripper gevonden is. Dit kan met het command `ls /dev | grep USB`. Waarschijnlijk is het zoiets als `/dev/ttyUSB0`.
```bash
# Zorg dat de gebruiker in de group doulout zit
sudo usermod -a -G dialout YOURUSERNAME

# Start de driver van de gripper
cd ~/catkin_wc/src/TNO_Project2_PCB/python_scripts/
python Robotiq2FGripperRtuNodepy controller_path

# Start interface voor de gripper
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```


# Gewrichten limieten
Gewricht | min rad | max rad | min deg | max deg 
--- | --- | --- | --- |---
Shoulder pan | -1.3 | 1.3 | -74.48 | 74.48
Shoulder lift | -1.7453 | -0.5235 | -100 | -30
Elbow joint | x | 3.15 | x | 180.48
Wrist1 | 2.356 | 6.28 | 134.99 | 359.82
Wrist2 | -3.15 | 3.15 | -180.48 | 180.48
Wrist3 | -3.15 | 3.15 | -180.48 | 180.48

Deze limieten hebben meerdere functies. De belangrijkste functie is dat het de oplosruimte verkleint, wat tot gevolg heeft dat de robot op een veilige manier van a naar b gaat. Een andere bijwerking van het verkleinen van de oplosruimte is dat het minder rekenkracht kost om een weg voor de robot te berekenen.
Let op, als de robot in een positie staat die niet binnen de hierboven genoemde limieten vallen zal de planner geen weg kunnen plannen. Zorg dus altijd dat de startpositie binnen de limieten valt.


# Debugging
Hieronder staan enkele problemen waar wij tegenaan gelopen zijn tijdens het maken van het project, met daaronder een oplossing die voor ont geholpen heeft.

Fout tijdens het importeren van pyassimp
```bash
# Mogelijke fix 1
pip install -U pyassimp

# Mogelijke fix 2
sudo dpkg --remove --force-depends python-pyassimp
sudo -H pip install pyassimp
```

Could not find a package configuration file provided by "industrial_msgs"
```bash
sudo apt install --reinstall ros-kinetic-industrial-msgs
```

The following packages have unmet dependencies:
    libaria-dev: Depends: libaria2v5 ...
```bash
sudo apt purge libaria-dev
```




# Autheurs:
- Martijn Schoenmaeker
- Dirk Brouwers
- Maik Sigmans
- Niels van den Bos
