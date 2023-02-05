#!/usr/bin/env python3

import rospy
from threading import Thread

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from ck_utilities_py_node.solenoid import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status, Fault, Health_Monitor_Control

import math

def publish_arm_base_link(degrees : float):
    transform = Transform()
    transform.linear.z = 0.24511 # Quarter of a meter up? - need to check cad for pivot point of base arm
    transform.angular.pitch = math.radians(degrees)

    transform_link = TransformLink("arm_base", "base_link") # originate from base link which is the center of the robot
    transform_link.set_transform(transform)
    transform_link.publish()

def publish_arm_upper_link(degrees : float):
    transform = Transform()
    transform.linear.z = 0.847725 # 0.87630

    transform.angular.pitch = math.radians(degrees)

    transform_link = TransformLink("arm_upper", "arm_base")
    transform_link.set_transform(transform)
    transform_link.publish()

def publish_arm_extender_link(degrees : float, extension : float):
    transform = Transform()
   # transform.linear.z = 0 #
    transform.angular.pitch = math.radians(degrees)
    transform.linear.z = extension * 0.3556 + 0.2032

    transform_link = TransformLink("arm_extender", "arm_upper")
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

    extension_solenoid = Solenoid("extension", SolenoidType.SINGLE)

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

            if control_sub.get().extend:
                extension_solenoid.set(SolenoidState.ON)
            else:
                extension_solenoid.set(SolenoidState.OFF)

        publish_arm_base_link(armBaseMaster.get_sensor_position() * 360.0)#armBaseMaster.get_sensor_position() * 360.0)  #MGT pretend this is 45 for now for demo purposes
        publish_arm_upper_link(90)#secondArmMaster.get_sensor_position() * 360.0)
        publish_arm_extender_link(secondArmMaster.get_sensor_position() * 360.0, True)#extension_solenoid.get() == SolenoidState.ON)


        master_sticky_faults = armBaseMaster.get_sticky_faults()
        slave_sticky_faults = armBaseSlave.get_sticky_faults()

        status_message = Arm_Status()
        secondArmMaster.get_sensor_position()
        status_message.arm_base_actual_position = 0
        armBaseMaster.get_sensor_position()
        status_message.arm_upper_actual_position = 0
        status_message.extended = extension_solenoid.get() == SolenoidState.ON
        status_message.left_arm_base_remote_loss_of_signal = master_sticky_faults.RemoteLossOfSignal
        status_message.right_arm_base_remote_loss_of_signal = slave_sticky_faults.RemoteLossOfSignal
        status_message.left_arm_base_reset_during_en = master_sticky_faults.ResetDuringEn
        status_message.right_arm_base_reset_during_en = slave_sticky_faults.ResetDuringEn
        status_message.left_arm_base_hardware_ESD_reset = master_sticky_faults.HardwareESDReset
        status_message.right_arm_base_hardware_ESD_reset = slave_sticky_faults.HardwareESDReset
        status_message.left_arm_base_supply_unstable = master_sticky_faults.SupplyUnstable
        status_message.right_arm_base_supply_unstable = slave_sticky_faults.SupplyUnstable
        status_publisher.publish(status_message)



        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    drivebase = Cube("robot_base", 1, "base_link")
    drivebase.set_scale(Scale(0.6096, 0.6096, 0.127))
    drivetrans = Transform()
    drivetrans.linear.z = 0.1143
    drivebase.set_transform(drivetrans)
    drivebase.set_color(Color(.5, 0, 1.0, 1.0))
    drivebase.publish()

    arm_cube = Cube("robot_parts", 1, "arm_base")
    arm_transform = Transform()
    arm_transform.linear.z = 0.56515
    arm_cube.set_transform(arm_transform)
    arm_cube.set_scale(Scale(0.0508, 0.508, 1.1303)) #44.5 tall, 20 apart
    arm_cube.set_color(Color(.7, .7, .7, 1.0))
    arm_cube.publish()

    arm_upper_cube = Cube("robot_parts", 2, "arm_upper")
    arm_upper_transform = Transform()
    arm_upper_transform.linear.z = 0.3048
    arm_upper_cube.set_transform(arm_upper_transform)
    arm_upper_cube.set_scale(Scale(0.0508, 0.381, 0.6096)) #24, 15, 2
    arm_upper_cube.set_color(Color(.7, .7, .7, 1.0))
    arm_upper_cube.publish()

    arm_extender_cube = Cube("robot_parts", 3, "arm_extender")
    arm_extender_transform = Transform()
    arm_extender_transform.linear.z = 0.2032 #12 in
    arm_extender_cube.set_transform(arm_extender_transform)
    arm_extender_cube.set_scale(Scale(0.0508, 0.304673, 0.4064)) #14 out (11.995, 16, 2)
    arm_extender_cube.set_color(Color(.7, .7, .7, 1.0))
    arm_extender_cube.publish()



    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
