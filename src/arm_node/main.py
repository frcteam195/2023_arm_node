#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread

from arm_node.arm_link import *
from ck_utilities_py_node.motor import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status, Fault, Health_Monitor_Control

def ros_func():
    global hmi_updates
    global robot_status

    control_sub = BufferedROSMsgHandlerPy(Arm_Control)
    control_sub.register_for_updates("ArmControl")
    status_pub = rospy.Publisher(name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)
    fault_pub = rospy.Publisher(name="HealthMonitorControl", data_class=Health_Monitor_Control, queue_size=50, tcp_nodelay=True)

    armBaseMotor = Motor(9, MotorType.TalonFX)
    armBaseMotor.set_defaults()
    armBaseMotor.set_neutral_mode(NeutralMode.Brake)
    armBaseMotor.set_forward_soft_limit(18000.0)
    armBaseMotor.set_reverse_soft_limit(0.0)
    armBaseMotor.apply()

    armUpperMotor = Motor(10, MotorType.TalonFX)
    armUpperMotor.set_defaults()
    armUpperMotor.set_neutral_mode(NeutralMode.Brake)
    armUpperMotor.set_forward_soft_limit(18000.0)
    armUpperMotor.set_reverse_soft_limit(0.0)
    armUpperMotor.apply()

    arm_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if control_sub.get() is not None:
            if robot_status.get_mode() == RobotMode.TELEOP:
                armBaseMotor.set(ControlMode.MOTION_MAGIC, control_sub.get().arm_base_requested_position, 0.0)
                armUpperMotor.set(ControlMode.MOTION_MAGIC, control_sub.get().arm_upper_requested_position, 0.0)
            else:
                armBaseMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                armUpperMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)

        arm_broadcaster.sendTransform(get_arm_transforms(armBaseMotor.get_sensor_position(), armUpperMotor.get_sensor_position()))

        pubmsg = Arm_Status()
        pubmsg.arm_base_actual_position = armUpperMotor.get_sensor_position()
        pubmsg.arm_upper_actual_position = armBaseMotor.get_sensor_position()
        status_pub.publish(pubmsg)
    
        example_fault = Fault()
        example_fault.code = "ArmStuck"
        example_fault.priority = 1

        fault_message = Health_Monitor_Control()
        fault_message.faults = [ example_fault ]
        fault_message.acknowledge = False
        fault_pub.publish(fault_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)