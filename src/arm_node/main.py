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
from arm_node.arm_simulation import ArmSimulation

def ros_func():
    global robot_status

    arm_simulation = ArmSimulation()

    control_sub = BufferedROSMsgHandlerPy(Arm_Control)
    control_sub.register_for_updates("ArmControl")
    status_publisher = rospy.Publisher(name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)

    baseArmMaster = Motor("baseArmMaster", MotorType.TalonFX)
    baseArmFollower = Motor("baseArmFollower", MotorType.TalonFX)
    upperArmMaster = Motor("upperArmMaster", MotorType.TalonFX)
    upperBaseFollower = Motor("upperArmFollower", MotorType.TalonFX)
    
    wristMotor = Motor("wristMotor", MotorType.TalonFX)

    extension_solenoid = Solenoid("extension", SolenoidType.SINGLE)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        arm_msg : Arm_Control = control_sub.get()
        if arm_msg is not None:
            if robot_status.get_mode() == RobotMode.TELEOP:
                #A little bit of trickery to allow inputs to be specified as unchanged (-20)

                if arm_msg.arm_base_requested_position > -10:
                    baseArmMaster.set(ControlMode.MOTION_MAGIC, arm_msg.arm_base_requested_position, 0.0)
                
                if arm_msg.arm_upper_requested_position > -10:
                    upperArmMaster.set(ControlMode.MOTION_MAGIC, arm_msg.arm_upper_requested_position, 0.0)

                if arm_msg.arm_wrist_requested_position > -10:
                    wristMotor.set(ControlMode.MOTION_MAGIC, arm_msg.arm_wrist_requested_position, 0.0)
                pass
            else:
                baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                pass


            if arm_msg.extend >= -10:
                if arm_msg.extend > 0:
                    extension_solenoid.set(SolenoidState.ON)
                else:
                    extension_solenoid.set(SolenoidState.OFF)

        arm_simulation.publish_arm_base_link(baseArmMaster.get_sensor_position() * 360.0)
        arm_simulation.publish_arm_upper_link(upperArmMaster.get_sensor_position() * 360.0)
        arm_simulation.publish_arm_extender_link(upperArmMaster.get_sensor_position() * 360.0, extension_solenoid.get() == SolenoidState.ON)


        master_sticky_faults = baseArmMaster.get_sticky_faults()
        follower_sticky_faults = baseArmFollower.get_sticky_faults()

        status_message = Arm_Status()
        status_message.arm_base_actual_position = baseArmMaster.get_sensor_position()
        status_message.arm_upper_actual_position = upperArmMaster.get_sensor_position()
        status_message.arm_wrist_actual_position = wristMotor.get_sensor_position()
        status_message.extended = extension_solenoid.get() == SolenoidState.ON
        status_message.left_arm_base_remote_loss_of_signal = master_sticky_faults.RemoteLossOfSignal
        status_message.right_arm_base_remote_loss_of_signal = follower_sticky_faults.RemoteLossOfSignal
        status_message.left_arm_base_reset_during_en = master_sticky_faults.ResetDuringEn
        status_message.right_arm_base_reset_during_en = follower_sticky_faults.ResetDuringEn
        status_message.left_arm_base_hardware_ESD_reset = master_sticky_faults.HardwareESDReset
        status_message.right_arm_base_hardware_ESD_reset = follower_sticky_faults.HardwareESDReset
        status_message.left_arm_base_supply_unstable = master_sticky_faults.SupplyUnstable
        status_message.right_arm_base_supply_unstable = follower_sticky_faults.SupplyUnstable
        status_publisher.publish(status_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
