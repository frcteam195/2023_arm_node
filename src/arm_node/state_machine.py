from arm_node.positions import *

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from enum import Enum
import rospy

# from arm_node.states.HomeState import HomeState


class ArmStateMachine(StateMachine):


    class States(Enum):
        HOME=1

        INTERMEDIATE_FRONT=2
        GROUND_CUBE_FRONT=3
        GROUND_CONE_FRONT=4
        GROUND_DEAD_CONE_FRONT=5
        SHELF_FRONT=6
        LOW_SCORE_FRONT=7
        MID_CUBE_FRONT=8
        HIGH_CUBE_FRONT=9
        MID_CONE_FRONT=10
        HIGH_CONE_FRONT=11

        INTERMEDIATE_BACK=12
        GROUND_CUBE_BACK=13
        GROUND_CONE_BACK=14
        GROUND_DEAD_CONE_BACK=15
        SHELF_BACK=16
        LOW_SCORE_BACL=17
        MID_CUBE_BACK=18
        HIGH_CUBE_BACK=19
        MID_CONE_BACK=20
        HIGH_CONE_BACK=21

        PENDULUM=12

    
    FRONT_STATES = [
        States.INTERMEDIATE_FRONT,
        States.GROUND_CUBE_FRONT,
        States.GROUND_CONE_FRONT,
        States.GROUND_DEAD_CONE_FRONT,
        States.SHELF_FRONT,
        States.LOW_SCORE_FRONT,
        States.MID_CUBE_FRONT,
        States.HIGH_CUBE_FRONT,
        States.MID_CONE_FRONT,
        States.HIGH_CONE_FRONT
    ]

    BACK_STATES = [
        States.INTERMEDIATE_BACK,
        States.GROUND_CUBE_BACK,
        States.GROUND_CONE_BACK,
        States.GROUND_DEAD_CONE_BACK,
        States.SHELF_BACK,
        States.LOW_SCORE_BACL,
        States.MID_CUBE_BACK,
        States.HIGH_CUBE_BACK,
        States.MID_CONE_BACK,
        States.HIGH_CONE_BACK,
    ]


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

        from arm_node.states.home_state import HomeState
        from arm_node.states.intermediate_front_state import IntermediateFrontState
        from arm_node.states.intemediate_back_state import IntermediateBackState
        from arm_node.states.shelf_state import ShelfState
        from arm_node.states.high_cube_state import HighCubeState

        states = {
            ArmStateMachine.States.HOME : HomeState(self),
            ArmStateMachine.States.INTERMEDIATE_FRONT : IntermediateFrontState(self),
            ArmStateMachine.States.INTERMEDIATE_BACK : IntermediateBackState(self),
            ArmStateMachine.States.SHELF_FRONT : ShelfState(self),
            ArmStateMachine.States.HIGH_CUBE_FRONT : HighCubeState(self),
            ArmStateMachine.States.HIGH_CUBE_BACK : HighCubeState(self, False),
            # ArmStateMachine.States.PENDULUM : ArmStateMachine.PendulumState(self),
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME

        super().__init__(states, state)
