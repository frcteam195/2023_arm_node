from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.StateMachine import StateMachine
from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from ck_ros_msgs_node.msg import Arm_Goal, Arm_Status

from frc_robot_utilities_py_node.odometry_helper import OdometryHelper

from actions_node.game_specific_actions.constant import RollerState

odom = OdometryHelper()

STATES_TO_MSG = {
    ArmStateMachine.States.HOME : Arm_Status.HOME,
    ArmStateMachine.States.INTERMEDIATE_FRONT : Arm_Status.INTERMEDIATE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_GROUND_FRONT : Arm_Status.INTERMEDIATE_GROUND_FRONT,
    ArmStateMachine.States.GROUND_CUBE_FRONT : Arm_Status.GROUND_CUBE_FRONT,
    ArmStateMachine.States.GROUND_CONE_FRONT : Arm_Status.GROUND_CONE_FRONT,
    ArmStateMachine.States.GROUND_DEAD_CONE_FRONT : Arm_Status.GROUND_CONE_FRONT,
    ArmStateMachine.States.SHELF_CUBE_FRONT : Arm_Status.SHELF_CUBE_FRONT,
    ArmStateMachine.States.LOW_CUBE_FRONT : Arm_Status.LOW_CUBE_FRONT,
    ArmStateMachine.States.SHELF_CONE_FRONT : Arm_Status.SHELF_CONE_FRONT,
    ArmStateMachine.States.LOW_CONE_FRONT : Arm_Status.LOW_CONE_FRONT,
    ArmStateMachine.States.MID_CUBE_FRONT : Arm_Status.MID_CUBE_FRONT,
    ArmStateMachine.States.HIGH_CUBE_FRONT : Arm_Status.HIGH_CUBE_FRONT,
    ArmStateMachine.States.MID_CONE_FRONT : Arm_Status.MID_CONE_FRONT,
    ArmStateMachine.States.HIGH_CONE_FRONT : Arm_Status.HIGH_CONE_FRONT,
    ArmStateMachine.States.PRE_SCORE_FRONT : Arm_Status.PRE_SCORE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_HIGH_CONE_FRONT : Arm_Status.INTERMEDIATE_HIGH_CONE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_FRONT : Arm_Status.INTERMEDIATE_HIGH_CUBE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_MID_CONE_FRONT : Arm_Status.INTERMEDIATE_MID_CONE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_MID_CUBE_FRONT : Arm_Status.INTERMEDIATE_MID_CUBE_FRONT,
    ArmStateMachine.States.INTERMEDIATE_BACK : Arm_Status.INTERMEDIATE_BACK,
    ArmStateMachine.States.INTERMEDIATE_GROUND_BACK : Arm_Status.INTERMEDIATE_GROUND_BACK,
    ArmStateMachine.States.GROUND_CUBE_BACK : Arm_Status.GROUND_CUBE_BACK,
    ArmStateMachine.States.GROUND_CONE_BACK : Arm_Status.GROUND_CONE_BACK,
    ArmStateMachine.States.GROUND_DEAD_CONE_BACK : Arm_Status.GROUND_DEAD_CONE_BACK,
    ArmStateMachine.States.SHELF_CUBE_BACK : Arm_Status.SHELF_CUBE_BACK,
    ArmStateMachine.States.LOW_CUBE_BACK : Arm_Status.LOW_CUBE_BACK,
    ArmStateMachine.States.SHELF_CONE_BACK : Arm_Status.SHELF_CONE_BACK,
    ArmStateMachine.States.LOW_CONE_BACK : Arm_Status.LOW_CONE_BACK,
    ArmStateMachine.States.MID_CUBE_BACK : Arm_Status.MID_CUBE_BACK,
    ArmStateMachine.States.HIGH_CUBE_BACK : Arm_Status.HIGH_CUBE_BACK,
    ArmStateMachine.States.MID_CONE_BACK : Arm_Status.MID_CONE_BACK,
    ArmStateMachine.States.HIGH_CONE_BACK : Arm_Status.HIGH_CONE_BACK,
    ArmStateMachine.States.PRE_SCORE_BACK : Arm_Status.PRE_SCORE_BACK,
    ArmStateMachine.States.INTERMEDIATE_HIGH_CONE_BACK : Arm_Status.INTERMEDIATE_HIGH_CONE_BACK,
    ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_BACK : Arm_Status.INTERMEDIATE_HIGH_CUBE_BACK,
    ArmStateMachine.States.INTERMEDIATE_MID_CONE_BACK : Arm_Status.INTERMEDIATE_MID_CONE_BACK,
    ArmStateMachine.States.INTERMEDIATE_MID_CUBE_BACK : Arm_Status.INTERMEDIATE_MID_CUBE_BACK,
    ArmStateMachine.States.STEAL_FRONT : Arm_Status.STEAL_FRONT,
    ArmStateMachine.States.STEAL_BACK : Arm_Status.STEAL_BACK,
    ArmStateMachine.States.PRE_DEAD_CONE_FRONT : Arm_Status.PRE_DEAD_CONE_FRONT,
    ArmStateMachine.States.PRE_DEAD_CONE_BACK : Arm_Status.PRE_DEAD_CONE_BACK,
    ArmStateMachine.States.SPORT_MODE_FRONT : Arm_Status.SPORT_MODE_FRONT,
    ArmStateMachine.States.SPORT_MODE_BACK : Arm_Status.SPORT_MODE_BACK
}

def transition_to_intermediate(is_front: bool) -> StateMachine.State:
    if is_front:
        return ArmStateMachine.States.INTERMEDIATE_FRONT
    else:
        return ArmStateMachine.States.INTERMEDIATE_BACK

def standard_step(arm: Arm, position: ArmPosition):
    """
    Standard arm control.
    """
    # if machine.baseMotor.is_at_setpoint(0.01) and machine.upperMotor.is_at_setpoint(0.01):
    #     machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.baseBrakeSolenoid.set(SolenoidState.OFF)
    #     machine.upperBrakeSolenoid.set(SolenoidState.OFF)
    # else:
    #     machine.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
    #     machine.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)

    # Apply brakes when within the setpoint, but only disable the brakes if it's much farther off.
    if arm.is_at_setpoint_raw(0.008, 0.008):
        arm.enable_brakes()
    elif not arm.is_at_setpoint_raw(0.01, 0.01):
        arm.disable_brakes()

    arm.set_motion_magic(position)

# def goal_is_high(machine: ArmStateMachine):
#     return machine.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED


# ARM_GOAL_DICT = {
#     Arm_Goal.HOME : ArmStateMachine.States.HOME,
#     Arm_Goal.GROUND_CUBE_FRONT : ArmStateMachine.States.GROUND_CUBE_FRONT,
#     Arm_Goal.GROUND_CUBE_BACK : ArmStateMachine.States.GROUND_CUBE_BACK,
#     Arm_Goal.GROUND_CONE_FRONT : ArmStateMachine.States.GROUND_CONE_FRONT,
#     Arm_Goal.GROUND_CONE_BACK : ArmStateMachine.States.GROUND_CONE_BACK,
#     Arm_Goal.SHELF_PICKUP_FRONT : ArmStateMachine.States.SHELF_FRONT,
#     Arm_Goal.SHELF_PICKUP_BACK : ArmStateMachine.States.SHELF_BACK,
#     Arm_Goal.LOW_SCORE_FRONT : ArmStateMachine.States.LOW_SCORE_FRONT,
#     Arm_Goal.LOW_SCORE_BACK : ArmStateMachine.States.LOW_SCORE_BACK,
#     Arm_Goal.MID_CONE_FRONT : ArmStateMachine.States.MID_CONE_FRONT,
#     Arm_Goal.MID_CONE_BACK : ArmStateMachine.States.MID_CONE_BACK,
#     Arm_Goal.MID_CUBE_FRONT : ArmStateMachine.States.MID_CUBE_FRONT,
#     Arm_Goal.MID_CUBE_BACK : ArmStateMachine.States.MID_CUBE_BACK,
#     Arm_Goal.HIGH_CONE_FRONT : ArmStateMachine.States.HIGH_CONE_FRONT,
#     Arm_Goal.HIGH_CONE_BACK : ArmStateMachine.States.HIGH_CONE_BACK,
#     Arm_Goal.HIGH_CUBE_FRONT : ArmStateMachine.States.HIGH_CUBE_FRONT,
#     Arm_Goal.HIGH_CUBE_BACK : ArmStateMachine.States.HIGH_CUBE_BACK,
#     Arm_Goal.GROUND_DEAD_CONE_FRONT : ArmStateMachine.States.GROUND_DEAD_CONE_FRONT,
#     Arm_Goal.GROUND_DEAD_CONE_BACK : ArmStateMachine.States.GROUND_DEAD_CONE_BACK,
# }

FRONT_GOALS = {
    Arm_Goal.HOME : ArmStateMachine.States.HOME,
    Arm_Goal.GROUND_CUBE : ArmStateMachine.States.GROUND_CUBE_FRONT,
    Arm_Goal.GROUND_CONE : ArmStateMachine.States.GROUND_CONE_FRONT,
    Arm_Goal.PRE_DEAD_CONE : ArmStateMachine.States.PRE_DEAD_CONE_FRONT,
    Arm_Goal.GROUND_DEAD_CONE : ArmStateMachine.States.GROUND_DEAD_CONE_FRONT,
    Arm_Goal.SHELF_CUBE_PICKUP : ArmStateMachine.States.SHELF_CUBE_FRONT,
    Arm_Goal.LOW_CUBE : ArmStateMachine.States.LOW_CUBE_FRONT,
    Arm_Goal.SHELF_CONE_PICKUP : ArmStateMachine.States.SHELF_CONE_FRONT,
    Arm_Goal.LOW_CONE : ArmStateMachine.States.LOW_CONE_FRONT,
    Arm_Goal.MID_CONE : ArmStateMachine.States.MID_CONE_FRONT,
    Arm_Goal.MID_CUBE : ArmStateMachine.States.MID_CUBE_FRONT,
    Arm_Goal.HIGH_CONE : ArmStateMachine.States.HIGH_CONE_FRONT,
    Arm_Goal.HIGH_CUBE : ArmStateMachine.States.HIGH_CUBE_FRONT,
    Arm_Goal.PRE_SCORE : ArmStateMachine.States.PRE_SCORE_FRONT,
    Arm_Goal.SPORT_MODE : ArmStateMachine.States.SPORT_MODE_FRONT
}

BACK_GOALS = {
    Arm_Goal.HOME : ArmStateMachine.States.HOME,
    Arm_Goal.GROUND_CUBE : ArmStateMachine.States.GROUND_CUBE_BACK,
    Arm_Goal.GROUND_CONE : ArmStateMachine.States.GROUND_CONE_BACK,
    Arm_Goal.PRE_DEAD_CONE : ArmStateMachine.States.PRE_DEAD_CONE_BACK,
    Arm_Goal.GROUND_DEAD_CONE : ArmStateMachine.States.GROUND_DEAD_CONE_BACK,
    Arm_Goal.SHELF_CUBE_PICKUP : ArmStateMachine.States.SHELF_CUBE_BACK,
    Arm_Goal.LOW_CUBE : ArmStateMachine.States.LOW_CUBE_BACK,
    Arm_Goal.SHELF_CONE_PICKUP : ArmStateMachine.States.SHELF_CONE_BACK,
    Arm_Goal.LOW_CONE : ArmStateMachine.States.LOW_CONE_BACK,
    Arm_Goal.MID_CONE : ArmStateMachine.States.MID_CONE_BACK,
    Arm_Goal.MID_CUBE : ArmStateMachine.States.MID_CUBE_BACK,
    Arm_Goal.HIGH_CONE : ArmStateMachine.States.HIGH_CONE_BACK,
    Arm_Goal.HIGH_CUBE : ArmStateMachine.States.HIGH_CUBE_BACK,
    Arm_Goal.PRE_SCORE : ArmStateMachine.States.PRE_SCORE_BACK,
    Arm_Goal.SPORT_MODE : ArmStateMachine.States.SPORT_MODE_BACK
}

SIDE_GOALS = {
    Arm_Goal.SIDE_FRONT : FRONT_GOALS,
    Arm_Goal.SIDE_BACK : BACK_GOALS
}

INVERTED_FRONT_GOALS = {v: k for k, v in FRONT_GOALS.items()}
INVERTED_BACK_GOALS = {v: k for k, v in BACK_GOALS.items()}

INVERTED_SIDES = {
    ArmStateMachine.GoalSides.HOME : Arm_Goal.SIDE_FRONT,
    ArmStateMachine.GoalSides.FRONT : Arm_Goal.SIDE_FRONT,
    ArmStateMachine.GoalSides.BACK : Arm_Goal.SIDE_BACK
}

INTAKE_GOALS = {
    Arm_Goal.INTAKE_OFF : RollerState.Off,
    Arm_Goal.INTAKE_CONE : RollerState.Intake_Cone,
    Arm_Goal.INTAKE_CUBE : RollerState.Intake_Cube,
    Arm_Goal.OUTTAKE_CONE : RollerState.Outtake_Cone,
    Arm_Goal.OUTTAKE_CUBE : RollerState.Outtake_Cube
}

def goal_msg_to_state(goal_msg: Arm_Goal):
    return SIDE_GOALS[goal_msg.goal_side][goal_msg.goal]



def state_to_msg(state: ArmStateMachine.States) -> Arm_Goal:
    goal_msg = Arm_Goal()

    goal_msg.goal_side = INVERTED_SIDES[ArmStateMachine.get_goal_side(state)]

    if goal_msg.goal_side is Arm_Goal.SIDE_FRONT:
        goal_msg.goal = INVERTED_FRONT_GOALS[state]
    else:
        goal_msg.goal = INVERTED_BACK_GOALS[state]

    return goal_msg

def intake_msg_to_state(goal_msg: Arm_Goal) -> RollerState:
    """
    Converts the arm goal message integer to an intake goal state.
    """
    return INTAKE_GOALS[goal_msg.intake_goal]
