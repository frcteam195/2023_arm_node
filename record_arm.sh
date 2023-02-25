#!/bin/sh
rosbag record --tcpnodelay --repeat-latched -O $1 /ArmStatus /ArmGoal /state_machines/ArmStateMachine /static_shapes /tf /tf_static /MotorControlFinal /MotorConfigurationFinal
