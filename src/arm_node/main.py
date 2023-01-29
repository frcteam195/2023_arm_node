#!/usr/bin/env python3

import rospy
from threading import Thread

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status, Fault, Health_Monitor_Control

import math

def publish_arm_base_link(degrees : float):
    transform = Transform()
    transform.linear.z = .25 # Quarter of a meter up? - need to check cad for pivot point of base arm
    transform.angular.pitch = math.radians(degrees)

    transform_link = TransformLink("arm_base", "base_link") # originate from base link which is the center of the robot
    transform_link.set_transform(transform)
    transform_link.publish()

def ros_func():
    global robot_status

    control_sub = BufferedROSMsgHandlerPy(Arm_Control)
    control_sub.register_for_updates("ArmControl")
    status_publisher = rospy.Publisher(name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)

    armBaseMaster = Motor("baseArmMaster", MotorType.TalonFX)
    armBaseSlave = Motor("baseArmSlave", MotorType.TalonFX)
    secondArmMaster = Motor("secondArmMaster", MotorType.TalonFX)
    secondBaseSlave = Motor("secondArmSlave", MotorType.TalonFX)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if control_sub.get() is not None:
            if robot_status.get_mode() == RobotMode.TELEOP:
                armBaseMaster.set(ControlMode.MOTION_MAGIC, control_sub.get().arm_base_requested_position, 0.0)
                secondArmMaster.set(ControlMode.MOTION_MAGIC, control_sub.get().arm_upper_requested_position, 0.0)
                pass
            else:
                armBaseMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                secondArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                pass

        publish_arm_base_link(45)#armBaseMaster.get_sensor_position() * 360.0)  #MGT pretend this is 45 for now for demo purposes

        status_message = Arm_Status()
        secondArmMaster.get_sensor_position()
        status_message.arm_base_actual_position = 0
        armBaseMaster.get_sensor_position()
        status_message.arm_upper_actual_position = 0
        status_publisher.publish(status_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    drivebase = Cube("robot_base", 1, "base_link")
    drivebase.set_scale(Scale(1.0, 1.0, 0.25))
    drivetrans = Transform()
    drivetrans.linear.z = 0.125
    drivebase.set_transform(drivetrans)
    drivebase.set_color(Color(.5, 0, 1.0, 1.0))
    drivebase.publish()

    arm_cube = Cube("robot_parts", 1, "arm_base")
    arm_transform = Transform()
    arm_transform.linear.z = 0.5
    arm_cube.set_transform(arm_transform)
    arm_cube.set_scale(Scale(0.1, 0.5, 1.0))
    arm_cube.set_color(Color(.7, .7, .7, 1.0))
    arm_cube.publish()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
