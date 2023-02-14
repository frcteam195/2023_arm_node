from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class ShelfState(StateMachine.State):

    def __init__(self, machine):
        self.machine: ArmStateMachine = machine

    def get_enum(self):
        return ArmStateMachine.States.SHELF_FRONT

    def entry(self):
        # print('Entering', self.get_enum())
        self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
        self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
            
    def step(self):
        # if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
        if self.machine.baseMotor.is_at_setpoint(0.01) and self.machine.upperMotor.is_at_setpoint(0.01):
            self.machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0)
            self.machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0)
            self.machine.baseBrakeSolenoid.set(SolenoidState.OFF)
            self.machine.upperBrakeSolenoid.set(SolenoidState.OFF)
        else:
            self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, POS_SHELF.base_position)
            self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, POS_SHELF.upper_position)

    def transition(self) -> str:
        if self.machine.goal_state is not self.get_enum():
            return ArmStateMachine.States.INTERMEDIATE_FRONT

        return self.get_enum()