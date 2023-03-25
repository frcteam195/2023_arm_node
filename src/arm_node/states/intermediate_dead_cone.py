
import numpy

from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine

TRANSITIONS = [
    ArmStateMachine.States.GROUND_CONE_BACK,
    ArmStateMachine.States.GROUND_CONE_FRONT,
    ArmStateMachine.States.GROUND_CUBE_BACK,
    ArmStateMachine.States.GROUND_CUBE_FRONT,
    ArmStateMachine.States.GROUND_DEAD_CONE_BACK,
    ArmStateMachine.States.GROUND_DEAD_CONE_FRONT,
]

class IntermediateDeadCone(StateMachine.State):

    def __init__(self, machine, arm, side=ArmStateMachine.GoalSides.FRONT):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.side: ArmStateMachine.GoalSides = side
        self.position: ArmPosition = POS_PRE_GROUND_DEAD_CONE

        if self.side is ArmStateMachine.GoalSides.BACK:
            self.position = mirror_position(self.position)

    def get_enum(self):
        if self.side is ArmStateMachine.GoalSides.FRONT:
            return ArmStateMachine.States.INTERMEDIATE_DEAD_CONE_FRONT
        else:
            return ArmStateMachine.States.INTERMEDIATE_DEAD_CONE_BACK

    def entry(self):
        self.arm.disable_brakes()
        self.arm.retract()

    def step(self):
        self.arm.set_motion_magic(self.position)


    def transition(self) -> Enum:
        if self.arm.is_at_setpoint_raw(0.06, 0.06) and self.arm.wrist_at_setpoint(0.04) and \
           self.side is ArmStateMachine.get_goal_side(self.machine.goal_state) and \
           self.machine.goal_state in TRANSITIONS:
            return self.machine.goal_state
        elif self.arm.is_at_setpoint_raw(0.025, 0.025) and self.arm.wrist_at_setpoint(0.04):
            return transition_to_intermediate(self.side is ArmStateMachine.GoalSides.FRONT)
        else:
            return self.get_enum()
