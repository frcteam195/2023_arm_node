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

class ArmStateMachine(StateMachine):


    class States(Enum):
        HOME=1

        INTERMEDIATE_FRONT=2
        INTERMEDIATE_GROUND_FRONT=3
        GROUND_CUBE_FRONT=4
        GROUND_CONE_FRONT=5
        GROUND_DEAD_CONE_FRONT=6
        SHELF_FRONT=7
        LOW_SCORE_FRONT=8
        MID_CUBE_FRONT=9
        HIGH_CUBE_FRONT=10
        MID_CONE_FRONT=11
        HIGH_CONE_FRONT=12
        PRE_SCORE_FRONT=13
        INTERMEDIATE_HIGH_CONE_FRONT=14
        INTERMEDIATE_HIGH_CUBE_FRONT=15
        INTERMEDIATE_MID_CONE_FRONT=16
        INTERMEDIATE_MID_CUBE_FRONT=17
        SPORT_MODE_FRONT=18


        INTERMEDIATE_BACK=19
        INTERMEDIATE_GROUND_BACK=20
        GROUND_CUBE_BACK=21
        GROUND_CONE_BACK=22
        GROUND_DEAD_CONE_BACK=23
        SHELF_BACK=24
        LOW_SCORE_BACK=25
        MID_CUBE_BACK=26
        HIGH_CUBE_BACK=27
        MID_CONE_BACK=28
        HIGH_CONE_BACK=29
        PRE_SCORE_BACK=30
        INTERMEDIATE_HIGH_CONE_BACK=31
        INTERMEDIATE_HIGH_CUBE_BACK=32
        INTERMEDIATE_MID_CONE_BACK=33
        INTERMEDIATE_MID_CUBE_BACK=34
        SPORT_MODE_BACK=35

        STEAL_FRONT=36
        STEAL_BACK=37

        PRE_DEAD_CONE_FRONT=38
        PRE_DEAD_CONE_BACK=39

        SIDEWAYS_DEAD_CONE_FRONT=40
        SIDEWAYS_DEAD_CONE_BACK=41
        # INTERMEDIATE_DEAD_CONE_FRONT=36
        # INTERMEDIATE_DEAD_CONE_BACK=37

    class GoalSides(Enum):
        HOME=1
        FRONT=2
        BACK=3

    FRONT_STATES = [
        States.INTERMEDIATE_FRONT,
        States.INTERMEDIATE_GROUND_FRONT,
        States.GROUND_CUBE_FRONT,
        States.GROUND_CONE_FRONT,
        States.PRE_DEAD_CONE_FRONT,
        States.GROUND_DEAD_CONE_FRONT,
        States.SHELF_FRONT,
        States.LOW_SCORE_FRONT,
        States.MID_CUBE_FRONT,
        States.HIGH_CUBE_FRONT,
        States.MID_CONE_FRONT,
        States.HIGH_CONE_FRONT,
        States.STEAL_FRONT,
        States.PRE_SCORE_FRONT,
        States.INTERMEDIATE_HIGH_CONE_FRONT,
        States.INTERMEDIATE_HIGH_CUBE_FRONT,
        States.INTERMEDIATE_MID_CONE_FRONT,
        States.INTERMEDIATE_MID_CUBE_FRONT,
        States.SPORT_MODE_FRONT,
        States.SIDEWAYS_DEAD_CONE_FRONT,
    ]

    BACK_STATES = [
        States.INTERMEDIATE_BACK,
        States.INTERMEDIATE_GROUND_BACK,
        States.GROUND_CUBE_BACK,
        States.GROUND_CONE_BACK,
        States.PRE_DEAD_CONE_BACK,
        States.GROUND_DEAD_CONE_BACK,
        States.SHELF_BACK,
        States.LOW_SCORE_BACK,
        States.MID_CUBE_BACK,
        States.HIGH_CUBE_BACK,
        States.MID_CONE_BACK,
        States.HIGH_CONE_BACK,
        States.STEAL_BACK,
        States.PRE_SCORE_BACK,
        States.INTERMEDIATE_HIGH_CONE_BACK,
        States.INTERMEDIATE_HIGH_CUBE_BACK,
        States.INTERMEDIATE_MID_CONE_BACK,
        States.INTERMEDIATE_MID_CUBE_BACK,
        States.SPORT_MODE_BACK,
        States.SIDEWAYS_DEAD_CONE_BACK,
    ]


    REDIRECTED_STATES = {
        States.GROUND_CONE_BACK : States.INTERMEDIATE_GROUND_BACK,
        States.GROUND_CONE_FRONT : States.INTERMEDIATE_GROUND_FRONT,
        States.GROUND_CUBE_BACK : States.INTERMEDIATE_GROUND_BACK,
        States.GROUND_CUBE_FRONT : States.INTERMEDIATE_GROUND_FRONT,
        States.SIDEWAYS_DEAD_CONE_BACK: States.INTERMEDIATE_GROUND_BACK,
        States.SIDEWAYS_DEAD_CONE_FRONT: States.INTERMEDIATE_GROUND_FRONT,
        States.HIGH_CONE_BACK : States.INTERMEDIATE_HIGH_CONE_BACK,
        States.HIGH_CONE_FRONT : States.INTERMEDIATE_HIGH_CONE_FRONT,
        States.HIGH_CUBE_BACK : States.INTERMEDIATE_HIGH_CUBE_BACK,
        States.HIGH_CUBE_FRONT : States.INTERMEDIATE_HIGH_CUBE_FRONT,
        States.MID_CONE_BACK : States.INTERMEDIATE_MID_CONE_BACK,
        States.MID_CONE_FRONT : States.INTERMEDIATE_MID_CONE_FRONT,
        States.MID_CUBE_BACK : States.INTERMEDIATE_MID_CUBE_BACK,
        States.MID_CUBE_FRONT : States.INTERMEDIATE_MID_CUBE_FRONT,
    }


    def __init__(self, arm: Arm):
        self.arm = arm

        from arm_node.states.home_state import HomeState
        from arm_node.states.intermediate_base_state import IntermediateBaseState
        from arm_node.states.intermediate_ground_state import IntermediateGroundState
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
        from arm_node.states.high_cone_intermediate import IntermediateHighConeState
        from arm_node.states.high_cube_intermediate import IntermediateHighCubeState
        from arm_node.states.mid_cone_intermediate import IntermediateMidConeState
        from arm_node.states.mid_cube_intermediate import IntermediateMidCubeState
        from arm_node.states.pre_dead_cone_state import PreDeadConeState
        from arm_node.states.sport_mode_state import SportModeState
        from arm_node.states.sideways_dead_cone_state import SidewaysDeadCone

        states = {
            ArmStateMachine.States.HOME : HomeState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_FRONT : IntermediateBaseState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_GROUND_FRONT : IntermediateGroundState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_BACK : IntermediateBaseState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.INTERMEDIATE_GROUND_BACK : IntermediateGroundState(self, arm, ArmStateMachine.GoalSides.BACK),
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
            ArmStateMachine.States.PRE_SCORE_BACK : PreScoreState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.STEAL_FRONT : StealState(self, arm),
            ArmStateMachine.States.STEAL_BACK : StealState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.INTERMEDIATE_HIGH_CONE_FRONT : IntermediateHighConeState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_HIGH_CONE_BACK : IntermediateHighConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_FRONT : IntermediateHighCubeState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_HIGH_CUBE_BACK : IntermediateHighCubeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.INTERMEDIATE_MID_CONE_FRONT : IntermediateMidConeState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_MID_CONE_BACK : IntermediateMidConeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.INTERMEDIATE_MID_CUBE_FRONT : IntermediateMidCubeState(self, arm),
            ArmStateMachine.States.INTERMEDIATE_MID_CUBE_BACK : IntermediateMidCubeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.SPORT_MODE_FRONT : SportModeState(self, arm, ArmStateMachine.GoalSides.FRONT),
            ArmStateMachine.States.SPORT_MODE_BACK : SportModeState(self, arm, ArmStateMachine.GoalSides.BACK),
            ArmStateMachine.States.PRE_DEAD_CONE_FRONT : PreDeadConeState(self, arm),
            ArmStateMachine.States.PRE_DEAD_CONE_BACK: PreDeadConeState(self, arm, ArmStateMachine.GoalSides.BACK), 
            ArmStateMachine.States.SIDEWAYS_DEAD_CONE_FRONT: SidewaysDeadCone(self, arm),
            ArmStateMachine.States.SIDEWAYS_DEAD_CONE_BACK: SidewaysDeadCone(self, arm, ArmStateMachine.GoalSides.BACK)
        }

        state = ArmStateMachine.States.HOME

        self.goal_state = ArmStateMachine.States.HOME

        self.wrist_goal = WristPosition.Zero

        self.intake_pinched = False

        super().__init__(states, state)

    def set_goal(self, new_goal):
        if self.goal_state is not new_goal:
            self.goal_state = new_goal

    def set_goals(self, arm_goal, wrist_goal):
        self.set_goal(arm_goal)
        self.wrist_goal = wrist_goal

    def goal_is_high(self) -> bool:
        return self.goal_state in ArmStateMachine.HIGH_INTERMEDIATE_NEEDED

    @staticmethod
    def get_goal_side(goal):
        if goal is ArmStateMachine.States.HOME or goal is ArmStateMachine.States.HOME:
            return ArmStateMachine.GoalSides.HOME
        if goal in ArmStateMachine.FRONT_STATES:
            return ArmStateMachine.GoalSides.FRONT
        elif goal in ArmStateMachine.BACK_STATES:
            return ArmStateMachine.GoalSides.BACK
