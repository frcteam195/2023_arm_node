#!/usr/bin/env python3

import rospy
from threading import Thread
from ck_utilities_py_node.motor import *
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from ck_utilities_py_node.solenoid import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Goal, Arm_Status, Fault, Health_Monitor_Control
from arm_node.arm_simulation import ArmSimulation
from ck_utilities_py_node.constraints import *
from arm_node.arm_constraints import ArmConstraints
from arm_node.state_machine import ArmStateMachine


def ros_func():
    global robot_status

    arm_simulation = ArmSimulation()

    control_sub = BufferedROSMsgHandlerPy(Arm_Control)
    control_sub.register_for_updates("ArmControl")
    goal_sub = BufferedROSMsgHandlerPy(Arm_Goal)
    goal_sub.register_for_updates("ArmGoal")
    status_publisher = rospy.Publisher(name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)

    baseArmMaster = Motor("baseArmMaster", MotorType.TalonFX)
    baseArmFollower = Motor("baseArmFollower", MotorType.TalonFX)
    upperArmMaster = Motor("upperArmMaster", MotorType.TalonFX)
    upperArmFollower = Motor("upperArmFollower", MotorType.TalonFX)

    wristMotor = Motor("wristMotor", MotorType.TalonFX)

    base_brake_solenoid = Solenoid("base_brake", SolenoidType.SINGLE)
    upper_brake_solenoid = Solenoid("upper_brake", SolenoidType.SINGLE)

    extension_solenoid = Solenoid("extension", SolenoidType.SINGLE)

    rate = rospy.Rate(20)

    state_machine = ArmStateMachine(baseArmMaster, upperArmMaster, wristMotor, base_brake_solenoid, upper_brake_solenoid, extension_solenoid)

    while not rospy.is_shutdown():
        robot_mode = robot_status.get_mode()
        goal: Arm_Goal = goal_sub.get()

        real_goal = None

        if goal is not None:
            if goal.goal == Arm_Goal.HOME:
                real_goal = ArmStateMachine.States.HOME
            elif goal.goal == Arm_Goal.SHELF_PICKUP_FRONT:
                real_goal = ArmStateMachine.States.SHELF_FRONT
            elif goal.goal == Arm_Goal.HIGH_CUBE_FRONT:
                real_goal = ArmStateMachine.States.HIGH_CUBE_FRONT

            # print('Setting goal to:', real_goal)
        else:
            real_goal = state_machine.goal_state


        if robot_mode == RobotMode.TELEOP:
            # print('is teleop')
            # baseArmMaster.set_neutral_mode(NeutralMode.Coast)
            # baseArmFollower.set_neutral_mode(NeutralMode.Coast)
            # upperArmMaster.set_neutral_mode(NeutralMode.Coast)
            # upperArmFollower.set_neutral_mode(NeutralMode.Coast)
            # baseArmMaster.apply()
            # upperArmMaster.apply()
            base_brake_solenoid.set(SolenoidState.ON)
            upper_brake_solenoid.set(SolenoidState.ON)
            # print("set on")
            # baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            # upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            # wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

            state_machine.goal_state = real_goal
            state_machine.step()

        elif robot_mode == RobotMode.DISABLED:
            # baseArmMaster.set_neutral_mode(NeutralMode.Brake)
            # baseArmFollower.set_neutral_mode(NeutralMode.Brake)
            # upperArmMaster.set_neutral_mode(NeutralMode.Brake)
            # upperArmFollower.set_neutral_mode(NeutralMode.Brake)
            # baseArmMaster.apply()
            # upperArmMaster.apply()
            base_brake_solenoid.set(SolenoidState.OFF)
            upper_brake_solenoid.set(SolenoidState.OFF)
            # print("set off")
            baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

        # extension_solenoid.set(SolenoidState.OFF)
        # if arm_msg is not None:
        #     if robot_status.get_mode() == RobotMode.TELEOP:
        #         #A little bit of trickery to allow inputs to be specified as unchanged (-20)

        #         if arm_msg.arm_base_requested_position > -10:
        #             baseArmMaster.set(ControlMode.MOTION_MAGIC, arm_msg.arm_base_requested_position, 0.0)

        #         if arm_msg.arm_upper_requested_position > -10:
        #             upperArmMaster.set(ControlMode.MOTION_MAGIC, arm_msg.arm_upper_requested_position, 0.0)

        #         if arm_msg.arm_wrist_requested_position > -10:
        #             wristMotor.set(ControlMode.MOTION_MAGIC, arm_msg.arm_wrist_requested_position, 0.0)
        #         pass
        #     else:
        #         baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
        #         upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
        #         wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
        #         pass


        #     if arm_msg.extend >= -10:
        #         if arm_msg.extend > 0:
        #             extension_solenoid.set(SolenoidState.ON)
        #         else:
        #             extension_solenoid.set(SolenoidState.OFF)

        arm_simulation.publish_arm_base_link(baseArmMaster.get_sensor_position() * 360.0)
        arm_simulation.publish_arm_upper_link(upperArmMaster.get_sensor_position() * 360.0 + 180.0)
        arm_simulation.publish_arm_extender_link(extension_solenoid.get() == SolenoidState.ON)
        arm_simulation.publish_arm_wrist_link(wristMotor.get_sensor_position() * 360.0)

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
