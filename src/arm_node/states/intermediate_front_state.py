from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class IntermediateFrontState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side
        self.default_position: ArmPosition = POS_INTERMEDIATE
        self.high_position: ArmPosition = POS_HIGH_INTERMEDIATE

        if side is ArmStateMachine.GoalSides.BACK:
            self.default_position = mirror_position(self.default_position)
            self.high_position = mirror_position(self.high_position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.INTERMEDIATE_FRONT
        else:
            return ArmStateMachine.States.INTERMEDIATE_BACK

    def entry(self):
        # print('Entering', self.get_enum())
        self.arm.disable_brakes()
        self.arm.retract()

    def step(self):
        # print('in transition')
        if self.machine.goal_is_high() or self.machine.prev_goal_was_high():
            self.arm.set_motion_magic_raw(self.high_position)
        else:
            self.arm.set_motion_magic_raw(self.default_position)

        self.arm.stow_wrist()

    def transition(self) -> Enum:
        if self.arm.is_at_setpoint_raw(0.06, 0.06) and self.arm.wrist_at_setpoint(0.04):
            if self.side is not ArmStateMachine.get_goal_side(self.machine.goal_state):
                return ArmStateMachine.States.FORCE_HOME

            return self.machine.goal_state
        elif self.machine.goal_same_side():
            return self.machine.goal_state

        return self.get_enum()
