from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.StateMachine import StateMachine
from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from ck_ros_msgs_node.msg import Arm_Goal

from actions_node.game_specific_actions.constant import WristPosition

def transition_to_intermediate(is_front: bool) -> StateMachine.State:
    if is_front:
        return ArmStateMachine.States.INTERMEDIATE_FRONT
    else:
        return ArmStateMachine.States.INTERMEDIATE_BACK

def standard_step(arm: Arm, position: ArmPosition, control_wrist=True):
    # if machine.baseMotor.is_at_setpoint(0.01) and machine.upperMotor.is_at_setpoint(0.01):
    #     machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.baseBrakeSolenoid.set(SolenoidState.OFF)
    #     machine.upperBrakeSolenoid.set(SolenoidState.OFF)
    # else:
    #     machine.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
    #     machine.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)
    if arm.is_at_setpoint_raw(0.005, 0.005):
        # arm.set_percent_output()
        arm.enable_brakes()
    else:
        arm.disable_brakes()

    # if wrist_position is not WristPosition.Unchanged:
    #     arm.set_wrist(wrist_position.value)

    if control_wrist:
        arm.set_wrist(arm.wrist_goal)

    arm.set_motion_magic_raw(position)

def goal_is_high(machine: ArmStateMachine):
    return machine.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED

def prev_goal_was_high(machine: ArmStateMachine):
    return machine.prev_goal in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED


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
    Arm_Goal.GROUND_DEAD_CONE : ArmStateMachine.States.GROUND_DEAD_CONE_FRONT,
    Arm_Goal.SHELF_PICKUP : ArmStateMachine.States.SHELF_FRONT,
    Arm_Goal.LOW_SCORE : ArmStateMachine.States.LOW_SCORE_FRONT,
    Arm_Goal.MID_CONE : ArmStateMachine.States.MID_CONE_FRONT,
    Arm_Goal.MID_CUBE : ArmStateMachine.States.MID_CUBE_FRONT,
    Arm_Goal.HIGH_CONE : ArmStateMachine.States.HIGH_CONE_FRONT,
    Arm_Goal.HIGH_CUBE : ArmStateMachine.States.HIGH_CUBE_FRONT,
}

BACK_GOALS = {
    Arm_Goal.HOME : ArmStateMachine.States.HOME,
    Arm_Goal.GROUND_CUBE : ArmStateMachine.States.GROUND_CUBE_BACK,
    Arm_Goal.GROUND_CONE : ArmStateMachine.States.GROUND_CONE_BACK,
    Arm_Goal.GROUND_DEAD_CONE : ArmStateMachine.States.GROUND_DEAD_CONE_BACK,
    Arm_Goal.SHELF_PICKUP : ArmStateMachine.States.SHELF_BACK,
    Arm_Goal.LOW_SCORE : ArmStateMachine.States.LOW_SCORE_BACK,
    Arm_Goal.MID_CONE : ArmStateMachine.States.MID_CONE_BACK,
    Arm_Goal.MID_CUBE : ArmStateMachine.States.MID_CUBE_BACK,
    Arm_Goal.HIGH_CONE : ArmStateMachine.States.HIGH_CONE_BACK,
    Arm_Goal.HIGH_CUBE : ArmStateMachine.States.HIGH_CUBE_BACK,
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

WRIST_GOALS = {
    Arm_Goal.WRIST_ZERO : WristPosition.Zero,
    Arm_Goal.WRIST_90 : WristPosition.Left_90,
    Arm_Goal.WRIST_180 : WristPosition.Left_180
}

def goal_msg_to_state(goal_msg: Arm_Goal):
    # if goal_msg.goal_side is Arm_Goal.SIDE_FRONT:
    #     return FRONT_GOALS[goal_msg.goal]
    # else:
    #     return BACK_GOALS[goal_msg.goal]
    return SIDE_GOALS[goal_msg.goal_side][goal_msg.goal]

def state_to_msg(state: ArmStateMachine.States) -> Arm_Goal:
    goal_msg = Arm_Goal()

    goal_msg.goal_side = INVERTED_SIDES[ArmStateMachine.get_goal_side(state)]

    if goal_msg.goal_side is Arm_Goal.SIDE_FRONT:
        goal_msg.goal = INVERTED_FRONT_GOALS[state]
    else:
        goal_msg.goal = INVERTED_BACK_GOALS[state]

    return goal_msg

def wrist_msg_to_state(goal_msg: Arm_Goal) -> WristPosition:
    return WRIST_GOALS[goal_msg.wrist_goal]
