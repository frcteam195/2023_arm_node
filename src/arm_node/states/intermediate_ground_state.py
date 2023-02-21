from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class IntermediateGroundState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side
        self.position: ArmPosition = POS_GROUND_INTERMEDIATE

        if self.side is ArmStateMachine.GoalSides.BACK:
            self.position = mirror_position(self.position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.INTERMEDIATE_GROUND_FRONT
        else:
            return ArmStateMachine.States.INTERMEDIATE_GROUND_BACK

    def entry(self):
        # print('Entering', self.get_enum())
        self.arm.disable_brakes()
        self.arm.retract()
        self.arm.stow_wrist()

    def step(self):
        # print('in transition')
        self.arm.set_motion_magic(self.position)

    def transition(self) -> Enum:
        if self.arm.is_at_setpoint_raw(0.06, 0.06) and self.arm.wrist_at_setpoint(0.04):
            if self.side is not ArmStateMachine.get_goal_side(self.machine.goal_state):
                return ArmStateMachine.States.FORCE_HOME
            return self.machine.goal_state
        elif self.side is ArmStateMachine.get_goal_side(self.machine.goal_state) and \
             self.arm.is_at_setpoint_raw(0.06, 1.0) and \
             abs(self.arm.upperMotor.get_sensor_position()) > abs(self.arm.upperMotor.get_setpoint()):
            return self.machine.goal_state
        elif self.machine.goal_same_side():
            return self.machine.goal_state

        return self.get_enum()
