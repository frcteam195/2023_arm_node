from arm_node.positions import *

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from enum import Enum
import rospy


class ArmStateMachine(StateMachine):


    class States(Enum):
        HOME=1,
        INTERMEDIATE_FRONT=2,
        GROUND_CUBE_FRONT=3,
        GROUND_CONE_FRONT=4,
        GROUND_DEAD_CODE_FRONT=5,
        SHELF_FRONT=6,
        LOW_SCORE_FRONT=7,
        MID_CUBE_FRONT=8,
        HIGH_CUBE_FRONT=9,
        MID_CONE_FRONT=10,
        HIGH_CONE_FRONT=11,
        PENDULUM=12


    class HomeState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine

        def get_enum(self):
            return ArmStateMachine.States.HOME

        def entry(self):
            print('Entering', self.get_enum())
            self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
            self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
            

        def step(self):
            if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
                self.machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0)
                self.machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0)
                self.machine.baseBrakeSolenoid.set(SolenoidState.OFF)
                self.machine.upperBrakeSolenoid.set(SolenoidState.OFF)
            else:
                self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, POS_HOME.base_position)
                self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, POS_HOME.upper_position)

        def transition(self) -> str:
            if self.machine.goal_state is not self.get_enum():
                return ArmStateMachine.States.INTERMEDIATE_FRONT

            return self.get_enum()

    class IntermediateFrontState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.INTERMEDIATE_FRONT

        def entry(self):
            # print('Entering', self.get_enum())
            # self.machine.extensionSolenoid.set(SolenoidState.OFF)
            self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
            self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
            self.machine.extensionSolenoid.set(SolenoidState.OFF)

        def step(self):
            self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, POS_INTERMEDIATE.base_position)
            self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, POS_INTERMEDIATE.upper_position)

        def transition(self) -> str:
            delta = 0.05
            if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
                return self.machine.goal_state

            return self.get_enum()

    class ShelfFrontState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine

        def get_enum(self):
            return ArmStateMachine.States.SHELF_FRONT

        def entry(self):
            # print('Entering', self.get_enum())
            self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
            self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
            

        def step(self):
            if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
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

    class HighCubeFrontState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine

        def get_enum(self):
            return ArmStateMachine.States.HIGH_CUBE_FRONT

        def entry(self):
            print('Entering', self.get_enum())
            # self.machine.extensionSolenoid.set(SolenoidState.ON)
            self.machine.baseBrakeSolenoid.set(SolenoidState.ON)
            self.machine.upperBrakeSolenoid.set(SolenoidState.ON)
            self.machine.extensionSolenoid.set(SolenoidState.ON)
            

        def step(self):
            if self.machine.baseMotor.is_at_setpoint(BASE_ALLOWED_DEVIATION) and self.machine.upperMotor.is_at_setpoint(UPPER_ALLOWED_DEVIATION):
                self.machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0)
                self.machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0)
                self.machine.baseBrakeSolenoid.set(SolenoidState.OFF)
                self.machine.upperBrakeSolenoid.set(SolenoidState.OFF)
            else:
                self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, POS_HIGH_CUBE.base_position)
                self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, POS_HIGH_CUBE.upper_position)

        def transition(self) -> str:
            if self.machine.goal_state is not self.get_enum():
                return ArmStateMachine.States.INTERMEDIATE_FRONT

            return self.get_enum()

    class PendulumState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
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

            self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, base_output)
            self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, upper_output)

        def transition(self) -> str:
            if robot_status.get_mode() == RobotMode.DISABLED:
                return ArmStateMachine.States.HOME

            return ArmStateMachine.States.PENDULUM

    def __init__(self, baseMotor: Motor, upperMotor: Motor, wristMotor: Motor, baseBrakeSolenoid: Solenoid, upperBrakeSolenoid: Solenoid, extensionSolenoid: Solenoid):
        self.baseMotor = baseMotor
        self.upperMotor = upperMotor
        self.wristMotor = wristMotor
        self.baseBrakeSolenoid = baseBrakeSolenoid
        self.upperBrakeSolenoid = upperBrakeSolenoid
        self.extensionSolenoid = extensionSolenoid

        states = {
            ArmStateMachine.States.HOME : ArmStateMachine.HomeState(self),
            ArmStateMachine.States.INTERMEDIATE_FRONT : ArmStateMachine.IntermediateFrontState(self),
            ArmStateMachine.States.SHELF_FRONT : ArmStateMachine.ShelfFrontState(self),
            ArmStateMachine.States.HIGH_CUBE_FRONT : ArmStateMachine.HighCubeFrontState(self),
            ArmStateMachine.States.PENDULUM : ArmStateMachine.PendulumState(self),
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME

        super().__init__(states, state)