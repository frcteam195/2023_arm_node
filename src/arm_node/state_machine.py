from arm_node.positions import *
from arm_node.arm import Arm

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from ck_utilities_py_node.StateMachine import StateMachine
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from enum import Enum
import rospy

from actions_node.game_specific_actions.constant import WristPosition

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
        PRE_SCORE_FRONT=12


        INTERMEDIATE_BACK=13
        GROUND_CUBE_BACK=14
        GROUND_CONE_BACK=15
        GROUND_DEAD_CONE_BACK=16
        SHELF_BACK=17
        LOW_SCORE_BACK=18
        MID_CUBE_BACK=19
        HIGH_CUBE_BACK=20
        MID_CONE_BACK=21
        HIGH_CONE_BACK=22
        PRE_SCORE_BACK=23


        FORCE_HOME=24
        PENDULUM=25

        STEAL_FRONT=26
        STEAL_BACK=27

    class GoalSides(Enum):
        HOME=1
        FRONT=2
        BACK=3

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
        States.HIGH_CONE_FRONT,
        States.STEAL_FRONT,
        States.PRE_SCORE_FRONT
    ]

    BACK_STATES = [
        States.INTERMEDIATE_BACK,
        States.GROUND_CUBE_BACK,
        States.GROUND_CONE_BACK,
        States.GROUND_DEAD_CONE_BACK,
        States.SHELF_BACK,
        States.LOW_SCORE_BACK,
        States.MID_CUBE_BACK,
        States.HIGH_CUBE_BACK,
        States.MID_CONE_BACK,
        States.HIGH_CONE_BACK,
        States.STEAL_BACK,
        States.PRE_SCORE_BACK
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
        from arm_node.states.high_cone_state import HighConeState
        from arm_node.states.mid_cone_state import MidConeState
        from arm_node.states.mid_cube_state import MidCubeState
        from arm_node.states.low_score_state import LowScoreState
        from arm_node.states.ground_cone_state import GroundConeState
        from arm_node.states.ground_cube_state import GroundCubeState
        from arm_node.states.ground_dead_cone_state import GroundDeadConeState
        from arm_node.states.steal_state import StealState
        from arm_node.states.pre_score_state import PreScoreState

        states = {
            ArmStateMachine.States.HOME : HomeState(self, arm),
            ArmStateMachine.States.FORCE_HOME : HomeState(self, arm, True),
            ArmStateMachine.States.INTERMEDIATE_FRONT : IntermediateFrontState(self, arm),
            # ArmStateMachine.States.INTERMEDIATE_BACK : IntermediateBackState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_BACK : IntermediateFrontState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.SHELF_FRONT : ShelfState(self, arm),
            ArmStateMachine.States.SHELF_BACK : ShelfState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.HIGH_CUBE_FRONT : HighCubeState(self, arm),
            ArmStateMachine.States.HIGH_CUBE_BACK : HighCubeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.HIGH_CONE_FRONT : HighConeState(self, arm),
            ArmStateMachine.States.HIGH_CONE_BACK : HighConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.MID_CONE_FRONT : MidConeState(self, arm),
            ArmStateMachine.States.MID_CONE_BACK : MidConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.MID_CUBE_FRONT : MidCubeState(self, arm),
            ArmStateMachine.States.MID_CUBE_BACK : MidCubeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.LOW_SCORE_FRONT : LowScoreState(self, arm),
            ArmStateMachine.States.LOW_SCORE_BACK : LowScoreState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.GROUND_CONE_FRONT : GroundConeState(self, arm),
            ArmStateMachine.States.GROUND_CONE_BACK : GroundConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.GROUND_CUBE_FRONT : GroundCubeState(self, arm),
            ArmStateMachine.States.GROUND_CUBE_BACK : GroundCubeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.GROUND_DEAD_CONE_FRONT : GroundDeadConeState(self, arm),
            ArmStateMachine.States.GROUND_DEAD_CONE_BACK : GroundDeadConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.PRE_SCORE_FRONT : PreScoreState(self, arm),
            ArmStateMachine.States.PRE_SCORE_BACK : PreScoreState(self, arm, ArmStateMachine.GoalSides.BACK)
            # ArmStateMachine.States.STEAL_FRONT : StealState(self, arm),
            # ArmStateMachine.States.STEAL_BACK : StealState(self, arm, ArmStateMachine.GoalSides.BACK),
            # ArmStateMachine.States.PENDULUM : ArmStateMachine.PendulumState(self),
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME
        self.prev_goal = self.goal_state

        self.wrist_goal = WristPosition.Zero

        super().__init__(states, state)

    def set_goal(self, new_goal):
        if self.goal_state is not new_goal:
            self.prev_goal = self.goal_state
            self.goal_state = new_goal

    def set_goals(self, arm_goal, wrist_goal):
        self.set_goal(arm_goal)
        self.wrist_goal = wrist_goal

    def goal_is_high(self) -> bool:
        return self.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED

    def prev_goal_was_high(self) -> bool:
        return self.prev_goal in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED

    def goal_same_side(self) -> bool:
        return ArmStateMachine.get_goal_side(self.goal_state) is ArmStateMachine.get_goal_side(self.prev_goal)

    @staticmethod
    def get_goal_side(goal):
        if goal is ArmStateMachine.States.HOME or goal is ArmStateMachine.States.FORCE_HOME:
            return ArmStateMachine.GoalSides.HOME
        if goal in ArmStateMachine.FRONT_STATES:
            return ArmStateMachine.GoalSides.FRONT
        elif goal in ArmStateMachine.BACK_STATES:
            return ArmStateMachine.GoalSides.BACK
