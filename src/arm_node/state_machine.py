from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from enum import Enum
import rospy


class ArmStateMachine(StateMachine):


    class States(Enum):
        HOME=1
        INTERMEDIATE_FRONT=2
        PENDULUM=3

    class HomeState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.HOME

        def entry(self):
            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            self.machine.baseMotor.set(ControlMode.PERCENT_OUTPUT, 0)
            self.machine.upperMotor.set(ControlMode.PERCENT_OUTPUT, 0)

        def transition(self) -> str:
            if robot_status.get_mode() == RobotMode.TELEOP:
                return ArmStateMachine.States.PENDULUM

            return ArmStateMachine.States.HOME

    class IntermediateFrontState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.INTERMEDIATE_FRONT

        def entry(self):

            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            print(self.get_enum())
            pass

        def transition(self) -> str:
            if rospy.Time().now().to_sec() - self.__entered_time > 1.0:
                return ArmStateMachine.States.HOME
            return ArmStateMachine.States.INTERMEDIATE_FRONT

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

            self.machine.upperMotor.set(ControlMode.POSITION, output)

        def transition(self) -> str:
            if robot_status.get_mode() == RobotMode.DISABLED:
                return ArmStateMachine.States.HOME

            return ArmStateMachine.States.PENDULUM

    def __init__(self, baseMotor: Motor, upperMotor: Motor, wristMotor: Motor, extensionSolenoid: Solenoid):
        self.baseMotor = baseMotor
        self.upperMotor = upperMotor
        self.wristMotor = wristMotor
        self.extensionSolenoid = extensionSolenoid

        states = {
            ArmStateMachine.States.HOME : ArmStateMachine.HomeState(self),
            ArmStateMachine.States.INTERMEDIATE_FRONT : ArmStateMachine.IntermediateFrontState(self),
            ArmStateMachine.States.PENDULUM : ArmStateMachine.PendulumState(self),
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME

        super().__init__(states, state)