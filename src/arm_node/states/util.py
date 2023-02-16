from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.StateMachine import StateMachine
from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from ck_ros_msgs_node.msg import Arm_Goal

def transition_to_intermediate(is_front: bool) -> StateMachine.State:
    if is_front:
        return ArmStateMachine.States.INTERMEDIATE_FRONT
    else:
        return ArmStateMachine.States.INTERMEDIATE_BACK

def standard_step(arm: Arm, position: ArmPosition):
    # if machine.baseMotor.is_at_setpoint(0.01) and machine.upperMotor.is_at_setpoint(0.01):
    #     machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)
    #     machine.baseBrakeSolenoid.set(SolenoidState.OFF)
    #     machine.upperBrakeSolenoid.set(SolenoidState.OFF)
    # else:
    #     machine.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
    #     machine.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)
    if arm.is_at_setpoint(0.005, 0.005):
        # arm.set_percent_output()
        arm.enable_brakes()

    arm.set_motion_magic(position)

def goal_is_high(machine: ArmStateMachine):
    return machine.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED

def prev_goal_was_high(machine: ArmStateMachine):
    return machine.prev_goal in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED


ARM_GOAL_DICT = {
    Arm_Goal.HOME : ArmStateMachine.States.HOME,
    Arm_Goal.GROUND_CUBE_FRONT : ArmStateMachine.States.GROUND_CUBE_FRONT,
    Arm_Goal.GROUND_CUBE_BACK : ArmStateMachine.States.GROUND_CUBE_BACK,
    Arm_Goal.GROUND_CONE_FRONT : ArmStateMachine.States.GROUND_CONE_FRONT,
    Arm_Goal.GROUND_CONE_BACK : ArmStateMachine.States.GROUND_CONE_BACK,
    Arm_Goal.SHELF_PICKUP_FRONT : ArmStateMachine.States.SHELF_FRONT,
    Arm_Goal.SHELF_PICKUP_BACK : ArmStateMachine.States.SHELF_BACK,
    Arm_Goal.LOW_SCORE_FRONT : ArmStateMachine.States.LOW_SCORE_FRONT,
    Arm_Goal.LOW_SCORE_BACK : ArmStateMachine.States.LOW_SCORE_BACK,
    Arm_Goal.MID_CONE_FRONT : ArmStateMachine.States.MID_CONE_FRONT,
    Arm_Goal.MID_CONE_BACK : ArmStateMachine.States.MID_CONE_BACK,
    Arm_Goal.MID_CUBE_FRONT : ArmStateMachine.States.MID_CUBE_FRONT,
    Arm_Goal.MID_CUBE_BACK : ArmStateMachine.States.MID_CUBE_BACK,
    Arm_Goal.HIGH_CONE_FRONT : ArmStateMachine.States.HIGH_CONE_FRONT,
    Arm_Goal.HIGH_CONE_BACK : ArmStateMachine.States.HIGH_CONE_BACK,
    Arm_Goal.HIGH_CUBE_FRONT : ArmStateMachine.States.HIGH_CUBE_FRONT,
    Arm_Goal.HIGH_CUBE_BACK : ArmStateMachine.States.HIGH_CUBE_BACK,
    Arm_Goal.GROUND_DEAD_CONE_FRONT : ArmStateMachine.States.GROUND_DEAD_CONE_FRONT,
    Arm_Goal.GROUND_DEAD_CONE_BACK : ArmStateMachine.States.GROUND_DEAD_CONE_BACK,
}

def goal_msg_to_state(goal_msg: Arm_Goal):
    return ARM_GOAL_DICT[goal_msg.goal]
