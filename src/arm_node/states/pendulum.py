import math

from arm_node.arm import Arm
from arm_node.positions import *
from arm_node.states.util import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class PendulumState(StateMachine.State):

    def __init__(self, machine, arm):
        self.machine: ArmStateMachine = machine
        self.arm: Arm = arm
        self.__entered_time = None

    def get_enum(self):
        return ArmStateMachine.States.PENDULUM

    def entry(self):
        self.__entered_time = rospy.Time().now().to_sec()
        pass

    def step(self):
        time = rospy.Time().now().to_sec() - self.__entered_time

        speed = 2

        base_range = (0.13 / 1.4)
        base_home = 0.0224
        base_output = base_range * math.sin(speed * time) - base_home

        upper_range = (0.6958007813 / 2 / 2.54)
        upper_home = 0.5
        upper_output = upper_range * math.sin(speed * time) - upper_home

        self.arm.set_motion_magic(ArmPosition(base_output, upper_output))

    def transition(self) -> Enum:
        if self.machine.goal_state is not self.get_enum():
            return ArmStateMachine.States.HOME

        return self.get_enum()