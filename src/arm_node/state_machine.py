from arm_node.positions import *
from arm_node.arm import Arm

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

        FORCE_HOME=22
        PENDULUM=23

    
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

    HIGH_INTERMEDIATE_NEEDED = [
        States.HIGH_CUBE_FRONT,
        States.HIGH_CUBE_BACK,
        States.HIGH_CONE_FRONT,
        States.HIGH_CONE_BACK
    ]


    def __init__(self, arm: Arm):
        self.arm = arm

        from arm_node.states.home_state import HomeState
        from arm_node.states.intermediate_front_state import IntermediateFrontState
        from arm_node.states.intemediate_back_state import IntermediateBackState
        from arm_node.states.shelf_state import ShelfState
        from arm_node.states.high_cube_state import HighCubeState

        states = {
            ArmStateMachine.States.HOME : HomeState(self, arm),
            ArmStateMachine.States.FORCE_HOME : HomeState(self, arm, True),
            ArmStateMachine.States.INTERMEDIATE_FRONT : IntermediateFrontState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_BACK : IntermediateBackState(self, arm),
            ArmStateMachine.States.SHELF_FRONT : ShelfState(self, arm),
            ArmStateMachine.States.HIGH_CUBE_FRONT : HighCubeState(self, arm),
            ArmStateMachine.States.HIGH_CUBE_BACK : HighCubeState(self, arm, False),
            # ArmStateMachine.States.PENDULUM : ArmStateMachine.PendulumState(self),
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME
        self.prev_goal = self.goal_state

        super().__init__(states, state)

    def set_goal(self, new_goal):
        if self.goal_state is not new_goal:
            self.prev_goal = self.goal_state
            self.goal_state = new_goal
