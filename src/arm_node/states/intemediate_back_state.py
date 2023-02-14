from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class IntermediateBackState(StateMachine.State):

    def __init__(self, machine, arm):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.position = mirror_position(POS_INTERMEDIATE)

    def get_enum(self):
        return ArmStateMachine.States.INTERMEDIATE_BACK

    def entry(self):
        print('Entering', self.get_enum())
        self.arm.disable_brakes()
        self.arm.retract()

    def step(self):
        print('in transition')
        if self.machine.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED or self.machine.prev_goal in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED:
            self.arm.set_motion_magic(mirror_position(POS_HIGH_INTERMEDIATE))
        else:
            self.arm.set_motion_magic(self.position)

    def transition(self) -> Enum:
        if self.arm.is_at_setpoint(BASE_ALLOWED_DEVIATION, UPPER_ALLOWED_DEVIATION):
            if self.machine.goal_state in ArmStateMachine.FRONT_STATES:
                return ArmStateMachine.States.FORCE_HOME

            return self.machine.goal_state

        return self.get_enum()
