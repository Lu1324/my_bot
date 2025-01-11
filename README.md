## Einführung
Projekt zur Kartografierung der Umgebung mit Hilfe eines Roboters 
Ausführbar mit ROS 2 Iron auf Ubuntu 22.06 (MATE)
ausführliche Dokumentation in Studienarbeit "Praktische Umsetzung eines ferngesteuerten
Roboterautos mit SLAM-System"

## Voraussetzungen
Arduino mit Code "ros_Arduino_motor_control"
Raspberry Pi + PC beide mit Ubuntu und ROS 2

## Launch
### Auf Raspberry Pi (Roboter) ausführen:

```
source /opt/ros/<distro>/setup.bash  # not needed if added to .bashrc
colcon build --symlink-install
source setup/install.bash
ros2 launch my_bot robot.launch.py
```

### auf PC ausführen:

```
source /opt/ros/<distro>/setup.bash # not needed if added to .bashrc
colcon build --symlink-install
source setup/install.bash
ros2 launch my_bot robot.launch.py
```