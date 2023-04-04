from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class HighCubeState(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side

        self.position: ArmPosition = POS_HIGH_CUBE

        if self.side is ArmStateMachine.GoalSides.BACK:
            self.position = mirror_position(self.position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.HIGH_CUBE_FRONT
        else:
            return ArmStateMachine.States.HIGH_CUBE_BACK

    def entry(self):
        self.arm.disable_brakes()
        self.arm.config_arm_normal()

    def step(self):
        self.arm.wrist_goal = WristPosition.Left_90
        standard_step(self.arm, self.position)

    def transition(self) -> Enum:
        if self.machine.goal_state is not self.get_enum():
            if self.side is ArmStateMachine.GoalSides.FRONT:
                return ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_FRONT
            else:
                return ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_BACK

        return self.get_enum()
