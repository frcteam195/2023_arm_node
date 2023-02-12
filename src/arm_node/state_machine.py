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
        PENDULUM_FRONT=3
        PENDULUM_BACK=4


    class HomeState(StateMachine.State):

        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.HOME

        def entry(self):
            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            print(self.get_enum())
            self.machine.baseMotor.set(ControlMode.MOTION_MAGIC, 0)
            self.machine.upperMotor.set(ControlMode.MOTION_MAGIC, 0)
            pass

        def transition(self) -> str:
            if robot_status.get_mode() == RobotMode.TELEOP:
                return ArmStateMachine.States.PENDULUM_FRONT

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

    class PendulumFrontState(StateMachine.State):
        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.PENDULUM_FRONT
    
        def entry(self):
            pass

        def step(self):
            time = rospy.Time().now().to_sec()
            output = 0.13 * math.sin(time) - 0.224

            self.machine.upperMotor.set(ControlMode.POSITION, output)

        def transition(self) -> str:
            if robot_status.get_mode() == RobotMode.DISABLED:
                return ArmStateMachine.States.HOME

            return ArmStateMachine.States.PENDULUM_FRONT

    class PendulumBackState(StateMachine.State):
        def __init__(self, machine):
            self.machine: ArmStateMachine = machine
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.PENDULUM_BACK
    
        def entry(self):
            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            if self.machine.baseMotor.get_raw_closed_loop_error() < 0.1:
                self.__entered_time = rospy.Time().now().to_sec()

        def transition(self) -> str:
            if self.__entered_time is not None and rospy.Time.now().to_sec() - self.__entered_time > 2.0:
                return ArmStateMachine.States.PENDULUM_FRONT

            return ArmStateMachine.States.PENDULUM_BACK


    def __init__(self, baseMotor: Motor, upperMotor: Motor, wristMotor: Motor, extensionSolenoid: Solenoid):
        self.baseMotor = baseMotor
        self.upperMotor = upperMotor
        self.wristMotor = wristMotor
        self.extensionSolenoid = extensionSolenoid

        states = {
            ArmStateMachine.States.HOME : ArmStateMachine.HomeState(self),
            ArmStateMachine.States.INTERMEDIATE_FRONT : ArmStateMachine.IntermediateFrontState(self),
            ArmStateMachine.States.PENDULUM_FRONT : ArmStateMachine.PendulumFrontState(self),
            ArmStateMachine.States.PENDULUM_BACK : ArmStateMachine.PendulumBackState(self)
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME

        super().__init__(states, state)