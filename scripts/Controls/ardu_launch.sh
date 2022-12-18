#!/bin/sh
# gnome-terminal --tab -- bash -ic "./QGroundControl\ .AppImage"
gnome-terminal --tab -- bash -ic "killall gzserver; killall gzclient; gazebo --verbose worlds/iris_arducopter_runway.world"
gnome-terminal --tab -- bash -ic "cd ardupilot/ArduCopter; ../Tools/autotest/sim_vehicle.py -L IITB -f gazebo-iris --console"
sleep 10
gnome-terminal --tab -- bash -ic "sb: sm; roslaunch mavros apm.launch"
source mavros_ws/devel/setup.bash
roslaunch apm.launch
