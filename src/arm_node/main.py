#!/usr/bin/env python3

import rospy
from threading import Thread

from ck_utilities_py_node.motor import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status, Fault, Health_Monitor_Control


def ros_func():
    global hmi_updates
    global robot_status

    control_sub = BufferedROSMsgHandlerPy(Arm_Control)
    control_sub.register_for_updates("ArmControl")
    status_publisher = rospy.Publisher(
        name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)
    fault_publisher = rospy.Publisher(
        name="HealthMonitorControl", data_class=Health_Monitor_Control, queue_size=50, tcp_nodelay=True)

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

        status_message = Arm_Status()
        secondArmMaster.get_sensor_position()
        status_message.arm_base_actual_position = 0
        armBaseMaster.get_sensor_position()
        status_message.arm_upper_actual_position = 0
        status_publisher.publish(status_message)

        example_fault = Fault()
        example_fault.code = "ArmStuck"
        example_fault.priority = 1

        fault_message = Health_Monitor_Control()
        fault_message.faults = [example_fault]
        fault_message.acknowledge = False
        fault_publisher.publish(fault_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
