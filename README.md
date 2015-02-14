VITO, the UNIPI Robot
======================

The UNIPI robot is composed of two Kuka LWR mounted on a torso. The torso is fixed. The end-effectors are several depending on the application, typically two Pisa/IIT Soft Hands.


0. Local dependencies
---------------

`git clone https://github.com/CentroEPiaggio/pisa-iit-soft-hand.git`

`git clone https://github.com/CentroEPiaggio/kuka-lwr.git`

`git clone https://github.com/CentroEPiaggio/kit-head.git`

`git clone https://github.com/CentroEPiaggio/gazebo_ros_pkgs.git`

`git clone https://github.com/CentroEPiaggio/ros_control.git`

IMPORTANT: if you want to simulate in gazebo, all repos must be checked out to the `multi-robot-test` branch.

For the real scenario, you will need:

`git clone https://github.com/CentroEPiaggio/calibration.git`

Note: you must fulfil dependencies therein.

1. Visualize
------------

`roslaunch vito_description display.launch`

By default, the simulation is paused at the begining to wait for all controllers and stuff load, otherwise, the robots move around without control. So, when the command above finishes, you must call in a different terminal the following:

`rosservice call /gazebo/unpause_physics`

See the "display.launch" file for more details and options, switch to real robots, select which part to simulate, load the moveit environment, etc.

2. Camera-robot calibration
---------------------------

Check instructions in the `calibration` package.