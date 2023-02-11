from ck_utilities_py_node.StateMachine import StateMachine
from enum import Enum
import rospy

class ArmStateMachine(StateMachine):

    class States(Enum):
        HOME=1
        INTERMEDIATE_FRONT=2

    class HomeState(StateMachine.State):

        def __init__(self):
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.HOME

        def entry(self):
            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            pass

        def transition(self) -> str:
            if rospy.Time().now().to_sec() - self.__entered_time > 1.0:
                return ArmStateMachine.States.INTERMEDIATE_FRONT
            return ArmStateMachine.States.HOME

    class IntermediateFrontState(StateMachine.State):

        def __init__(self):
            self.__entered_time = None

        def get_enum(self):
            return ArmStateMachine.States.INTERMEDIATE_FRONT

        def entry(self):
            self.__entered_time = rospy.Time().now().to_sec()

        def step(self):
            pass

        def transition(self) -> str:
            if rospy.Time().now().to_sec() - self.__entered_time > 1.0:
                return ArmStateMachine.States.HOME
            return ArmStateMachine.States.INTERMEDIATE_FRONT


    def initialize_states(self):
        self.states = {
            ArmStateMachine.States.HOME : ArmStateMachine.HomeState(),
            ArmStateMachine.States.INTERMEDIATE_FRONT : ArmStateMachine.IntermediateFrontState()
        }

        self.state = ArmStateMachine.States.HOME