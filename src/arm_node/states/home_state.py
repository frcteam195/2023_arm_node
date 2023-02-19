from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class HomeState(StateMachine.State):

    def __init__(self, machine, arm, force=False):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.force = force
        self.position: ArmPosition = POS_HOME

    def get_enum(self):
        return ArmStateMachine.States.HOME

    def entry(self):
        self.arm.disable_brakes()
        self.machine.wrist_goal = WristPosition.Zero

    def step(self):
        standard_step(self.arm, self.position, False)

    def transition(self) -> Enum:
        if self.machine.goal_state is not self.get_enum():
            new_state = transition_to_intermediate(self.machine.goal_state in ArmStateMachine.FRONT_STATES)

            if self.force:
                if self.arm.is_at_setpoint_raw(0.01, 0.01):
                    return new_state
            else:
                return new_state

        return self.get_enum()
