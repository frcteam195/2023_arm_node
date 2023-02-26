import numpy

from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine

TRANSITIONS = [
    ArmStateMachine.States.HIGH_CUBE_FRONT,
    ArmStateMachine.States.HIGH_CUBE_BACK,
]

class IntermediateHighCubeState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side
        self.default_position: ArmPosition = POS_HIGH_CUBE_EXTENSION_INTERMEDIATE

        if side is ArmStateMachine.GoalSides.BACK:
            self.default_position = mirror_position(self.default_position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_FRONT
        else:
            return ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_BACK

    def entry(self):
        if self.machine.goal_state not in TRANSITIONS:
            self.arm.config_arm_slow()
            self.arm.config_lower_arm_fast()
            self.default_position: ArmPosition = POS_HIGH_CUBE_RETRACTION_INTERMEDIATE
        elif self.machine.goal_state in TRANSITIONS:
            self.arm.config_arm_slow()
            self.arm.config_lower_arm_normal()
            self.default_position: ArmPosition = POS_HIGH_CUBE_EXTENSION_INTERMEDIATE

        if self.side is ArmStateMachine.GoalSides.BACK:
            self.default_position = mirror_position(self.default_position)

        self.arm.disable_brakes()

        self.arm.retract()

    def step(self):
        self.arm.set_motion_magic(self.default_position)

    def end(self):
        self.arm.config_lower_arm_normal()
        self.arm.config_arm_fast()

    def transition(self) -> Enum:
        if self.arm.is_at_setpoint_raw(0.015, 1) and \
           self.side is ArmStateMachine.get_goal_side(self.machine.goal_state) and \
           self.machine.goal_state in TRANSITIONS:
            return self.machine.goal_state
        elif self.arm.is_at_setpoint_raw(0.06, 0.06) and self.machine.goal_state not in TRANSITIONS:
            return transition_to_intermediate(self.side is ArmStateMachine.GoalSides.FRONT)
        return self.get_enum()
