from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class HighConeState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side

        self.position: ArmPosition = POS_HIGH_CONE

        if self.side is ArmStateMachine.GoalSides.BACK:
            self.position = mirror_position(self.position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.HIGH_CONE_FRONT
        else:
            return ArmStateMachine.States.HIGH_CONE_BACK

    def entry(self):
        self.arm.disable_brakes()
        self.arm.extend()
            
    def step(self):
        standard_step(self.arm, self.position)
        # if (self.arm.is_at_setpoint_raw(0.01, 0.01)):
        #     self.arm.extend()

    def transition(self) -> Enum:
        if self.machine.goal_state is not self.get_enum():
            return ArmStateMachine.States.PRE_SCORE_FRONT if self.side is ArmStateMachine.GoalSides.FRONT else ArmStateMachine.States.PRE_SCORE_BACK
            #return transition_to_intermediate(self.side is ArmStateMachine.GoalSides.FRONT)

        return self.get_enum()
