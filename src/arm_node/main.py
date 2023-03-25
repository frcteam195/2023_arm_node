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
from arm_node.state_machine import ArmStateMachine
from arm_node.positions import *
from arm_node.arm import Arm
from arm_node.states.util import goal_msg_to_state, intake_msg_to_state, state_to_msg, STATES_TO_MSG
from limelight_vision_node.msg import Limelight, Limelight_Control
from actions_node.game_specific_actions.constant import RollerState
# import cProfile


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
    intakeMotor = Motor("intake", MotorType.TalonFX)

    base_brake_solenoid = Solenoid("base_brake", SolenoidType.SINGLE)
    upper_brake_solenoid = Solenoid("upper_brake", SolenoidType.SINGLE)

    extension_solenoid = Solenoid("extension", SolenoidType.SINGLE)

    rate = rospy.Rate(100)

    arm = Arm(baseArmMaster, upperArmMaster, wristMotor, intakeMotor, base_brake_solenoid, upper_brake_solenoid, extension_solenoid, POS_HOME_CONE, None, None)

    state_machine = ArmStateMachine(arm)

    intake_goal = RollerState.Off

    frame_count = 0

    while not rospy.is_shutdown():
        robot_mode = robot_status.get_mode()
        goal_msg: Arm_Goal = goal_subscriber.get()

        arm_goal = None

        if goal_msg is not None:
            arm_goal = goal_msg_to_state(goal_msg)
            intake_goal = intake_msg_to_state(goal_msg)
        else:
            arm_goal = state_machine.goal_state

        if robot_mode in (RobotMode.TELEOP, RobotMode.AUTONOMOUS):
            state_machine.set_goal(arm_goal)
            arm.control_intake(intake_goal) # TODO: Custom speeds.
            state_machine.step()
        elif robot_mode == RobotMode.DISABLED:
            base_brake_solenoid.set(SolenoidState.OFF)
            upper_brake_solenoid.set(SolenoidState.OFF)
            extension_solenoid.set(SolenoidState.OFF)
            baseArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            upperArmMaster.set(ControlMode.PERCENT_OUTPUT, 0.0)
            wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
            intakeMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

        status_message : Arm_Status = arm.get_status()
        status_message.goal = state_to_msg(state_machine.goal_state)
        status_message.state = STATES_TO_MSG[state_machine.state]
        status_message.arm_at_setpoint = state_machine.goal_state == state_machine.state and arm.is_at_setpoint_raw(0.008, 0.008, 0.008)
        status_message.arm_base_deviation_deg = arm.get_base_deviation_deg()
        status_message.arm_upper_deviation_deg = arm.get_upper_deviation_deg()
        status_message.arm_wrist_deviation_deg = arm.get_wrist_deviation_deg()
        status_message.is_node_alive = True
        status_publisher.publish(status_message)

        if frame_count % 5 == 0:
            #20Hz Run
            arm_simulation.publish_arm_base_link(baseArmMaster.get_sensor_position() * 360.0)
            arm_simulation.publish_arm_upper_link(upperArmMaster.get_sensor_position() * 360.0)
            arm_simulation.publish_arm_extender_link(extension_solenoid.get() == SolenoidState.ON)
            arm_simulation.publish_arm_wrist_link(wristMotor.get_sensor_position() * 360.0)

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

            limelight_publisher.publish(limelight_control_msg)

        frame_count += 1
        frame_count = frame_count % 100

        rate.sleep()

def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    # profiler = cProfile.Profile()
    # profiler.enable()

    rospy.spin()
    t1.join(5)

    # profiler.disable()
    # profiler.dump_stats("/mnt/working/arm_node.stats")
