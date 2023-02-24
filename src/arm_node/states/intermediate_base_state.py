import numpy

from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine



class IntermediateBaseState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side
        self.default_position: ArmPosition = POS_INTERMEDIATE

        if side is ArmStateMachine.GoalSides.BACK:
            self.default_position = mirror_position(self.default_position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.INTERMEDIATE_FRONT
        else:
            return ArmStateMachine.States.INTERMEDIATE_BACK

    def entry(self):
        self.arm.disable_brakes()
        self.arm.retract()

    def step(self):
        self.arm.set_motion_magic(self.default_position)

        if self.side is not ArmStateMachine.get_goal_side(self.machine.goal_state):
            self.arm.stow_wrist()

    def end(self):
        self.arm.config_arm_normal()

    def transition(self) -> Enum:
        if self.arm.is_at_setpoint_raw(0.06, 0.06) and self.arm.wrist_at_setpoint(0.04):
            if self.side is not ArmStateMachine.get_goal_side(self.machine.goal_state):
                return ArmStateMachine.States.HOME
            if self.machine.goal_state in ArmStateMachine.REDIRECTED_STATES:
                return ArmStateMachine.REDIRECTED_STATES[self.machine.goal_state]
            return self.machine.goal_state
        elif self.side is ArmStateMachine.get_goal_side(self.machine.goal_state) and \
             self.arm.is_at_setpoint_raw(0.06, 1.0) and \
             numpy.sign(self.arm.upperMotor.get_sensor_position()) == numpy.sign(self.arm.upperMotor.get_setpoint()) and \
             abs(self.arm.upperMotor.get_sensor_position()) > abs(self.arm.upperMotor.get_setpoint()):
            if self.machine.goal_state in ArmStateMachine.REDIRECTED_STATES:
                return ArmStateMachine.REDIRECTED_STATES[self.machine.goal_state]
            return self.machine.goal_state

        return self.get_enum()

