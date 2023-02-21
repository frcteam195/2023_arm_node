from threading import Thread
import rospy

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from ck_utilities_py_node.solenoid import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Goal, Arm_Status, Intake_Status
from arm_node.arm_simulation import ArmSimulation
from ck_utilities_py_node.constraints import *
from arm_node.arm_constraints import ArmConstraints
from arm_node.state_machine import ArmStateMachine
from arm_node.positions import *
from arm_node.arm import Arm
from arm_node.states.util import goal_msg_to_state, state_to_msg, wrist_msg_to_state
from limelight_vision_node.msg import Limelight, Limelight_Control
from actions_node.game_specific_actions.constant import WristPosition


def ros_func():
    arm_simulation = ArmSimulation()

    goal_subscriber = BufferedROSMsgHandlerPy(Arm_Goal)
    goal_subscriber.register_for_updates("ArmGoal")

    intake_subscriber = BufferedROSMsgHandlerPy(Intake_Status)
    intake_subscriber.register_for_updates("IntakeStatus")

    status_publisher = rospy.Publisher(name="ArmStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)
    limelight_publisher = rospy.Publisher(name="LimelightControl", data_class=Limelight_Control, queue_size=50, tcp_nodelay=True)
    

    baseArmMaster = Motor("baseArmMaster", MotorType.TalonFX)
    baseArmFollower = Motor("baseArmFollower", MotorType.TalonFX)
    upperArmMaster = Motor("upperArmMaster", MotorType.TalonFX)
    upperArmFollower = Motor("upperArmFollower", MotorType.TalonFX)

    wristMotor = Motor("wristMotor", MotorType.TalonFX)

    base_brake_solenoid = Solenoid("base_brake", SolenoidType.SINGLE)
    upper_brake_solenoid = Solenoid("upper_brake", SolenoidType.SINGLE)

    extension_solenoid = Solenoid("extension", SolenoidType.SINGLE)

    rate = rospy.Rate(100)

    arm = Arm(baseArmMaster, upperArmMaster, wristMotor, base_brake_solenoid, upper_brake_solenoid, extension_solenoid, POS_HOME, None, None)

    state_machine = ArmStateMachine(arm)


    while not rospy.is_shutdown():
        robot_mode = robot_status.get_mode()
        goal_msg: Arm_Goal = goal_subscriber.get()

        arm_goal = None
        wrist_goal = None

        if goal_msg is not None:
            arm_goal = goal_msg_to_state(goal_msg)
            wrist_goal = wrist_msg_to_state(goal_msg)
            # print('Setting goal to:', real_goal)
        else:
            arm_goal = state_machine.goal_state
            wrist_goal = state_machine.wrist_goal

        # Update pinched state.
        intake_status_message = intake_subscriber.get()
        if intake_status_message is not None:
            state_machine.intake_pinched = intake_status_message.pinched

        # Flip limelight depending on arm position.
        limelight = Limelight()
        limelight.ledMode = 0
        limelight.camMode = 0
        limelight.stream = 0
        limelight.snapshot = 0
        limelight.name = "limelight-arm"
        limelight.pipeline = 1 if upperArmMaster.get_sensor_position() * 360.0 < 5.0 else 0

        limelight_control_msg = Limelight_Control()
        limelight_control_msg.limelights.append(limelight)

        if robot_mode in (RobotMode.TELEOP, RobotMode.AUTONOMOUS):
            state_machine.set_goal(arm_goal)
            arm.wrist_goal = wrist_goal
            state_machine.step()

        elif robot_mode == RobotMode.DISABLED:
            base_brake_solenoid.set(SolenoidState.OFF)
            upper_brake_solenoid.set(SolenoidState.OFF)
            extension_solenoid.set(SolenoidState.OFF)
            baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

        arm_simulation.publish_arm_base_link(baseArmMaster.get_sensor_position() * 360.0)
        arm_simulation.publish_arm_upper_link(upperArmMaster.get_sensor_position() * 360.0)
        arm_simulation.publish_arm_extender_link(extension_solenoid.get() == SolenoidState.ON)
        arm_simulation.publish_arm_wrist_link(wristMotor.get_sensor_position() * 360.0)

        # master_sticky_faults = baseArmMaster.get_sticky_faults()
        # follower_sticky_faults = baseArmFollower.get_sticky_faults()

        # status_message = Arm_Status()
        # status_message.arm_base_actual_position = baseArmMaster.get_sensor_position()
        # status_message.arm_upper_actual_position = upperArmMaster.get_sensor_position()
        # status_message.arm_wrist_actual_position = wristMotor.get_sensor_position()
        # status_message.extended = extension_solenoid.get() == SolenoidState.ON
        # status_message.left_arm_base_remote_loss_of_signal = master_sticky_faults.RemoteLossOfSignal
        # status_message.right_arm_base_remote_loss_of_signal = follower_sticky_faults.RemoteLossOfSignal
        # status_message.left_arm_base_reset_during_en = master_sticky_faults.ResetDuringEn
        # status_message.right_arm_base_reset_during_en = follower_sticky_faults.ResetDuringEn
        # status_message.left_arm_base_hardware_ESD_reset = master_sticky_faults.HardwareESDReset
        # status_message.right_arm_base_hardware_ESD_reset = follower_sticky_faults.HardwareESDReset
        # status_message.left_arm_base_supply_unstable = master_sticky_faults.SupplyUnstable
        # status_message.right_arm_base_supply_unstable = follower_sticky_faults.SupplyUnstable
        # status_publisher.publish(status_message)

        status_message = arm.get_status()
        status_message.goal = state_to_msg(state_machine.goal_state)
        status_message.arm_at_setpoint = state_machine.goal_state == state_machine.state and arm.is_at_setpoint_raw(0.007, 0.007)
        status_publisher.publish(status_message)

        limelight_publisher.publish(limelight_control_msg)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
