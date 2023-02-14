from arm_node.positions import *
from arm_node.state_machine import ArmStateMachine

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine


class IntermediateBackState(StateMachine.State):

    def __init__(self, machine):
        self.machine: ArmStateMachine = machine
        self.position = mirror_position(POS_INTERMEDIATE)

    def get_enum(self):
        return ArmStateMachine.States.INTERMEDIATE_BACK

    def entry(self):
        print('Entering', self.get_enum())
        # self.machine.extensionSolenoid.set(SolenoidState.OFF)
        self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
        self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
        self.machine.extensionSolenoid.set(SolenoidState.OFF)

    def step(self):
        print('in transition')
        self.machine.extensionSolenoid.set(SolenoidState.OFF)
        self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, self.position.base_position)
        self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, self.position.upper_position)

    def transition(self) -> str:
        if self.machine.goal_state in ArmStateMachine.FRONT_STATES:
            return ArmStateMachine.States.INTERMEDIATE_FRONT
        # if self.machine.goal_state in ArmStateMachine.BACK_STATES:
        #     return self.machine.goal_state
        # if self.machine.baseMotor.is_at_setpoint(0.01) and self.machine.upperMotor.is_at_setpoint(0.01):
        # if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
        #     print('ready for goal')
        #     return self.machine.goal_state

        return self.get_enum()