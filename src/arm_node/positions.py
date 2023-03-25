"""
Defines all the arm positions in degrees.
"""

from dataclasses import dataclass
import rospy

@dataclass
class ArmPosition:
    """
    Class containing the base and upper arm position.
    """
    base_position: float = 0.0
    upper_position: float = 0.0
    wrist_position: float = 0.0
    base_position_rear: float = 0.0
    upper_position_rear: float = 0.0
    wrist_position_rear: float = 0.0


POS_HOME_CUBE = ArmPosition(0.0, 0.0, 118.916)
POS_HOME_CONE = ArmPosition(0.0, 0.0, 118.916)

POS_STEAL = ArmPosition(16.70, 82.08, 0.0, -16.70, -82.08, 0.0)  # Fake

POS_GROUND_CUBE = ArmPosition(5.06, 33.0, 0.0, -5.06, -33.0, 0.0)
POS_GROUND_CONE = ArmPosition(-3.55, 32.16, 0.0, 4.55, -30.16, 0.0)
POS_PRE_GROUND_DEAD_CONE = ArmPosition(21.38, 40.68, -21.38, -42.0)
POS_GROUND_DEAD_CONE = ArmPosition(40, 33, 0.0, -40, -36, 0.0)

POS_SHELF_CUBE = ArmPosition(-14.98, 76.9, 0.0, 17.98, -74.1, 0.0)
POS_SHELF_CONE = ArmPosition(-14.98, 76.9, 0.0, 17.98, -74.1, 0.0)

POS_LOW_CUBE = ArmPosition(21.38, 40.68, 0.0, -21.38, -40.68, 0.0)
POS_MID_CUBE = ArmPosition(0, 68.88, 0.0, 0, -68.88, 0.0)
POS_HIGH_CUBE = ArmPosition(16, 111.0, 0.0, -16, -111.0, 0.0)
POS_LOW_CONE = ArmPosition(21.38, 40.68, 0.0, -21.38, -40.68, 0.0)
POS_MID_CONE = ArmPosition(-9.5, 81.73, 0.0, 6.0, -81.73, 0.0)
POS_HIGH_CONE = ArmPosition(19.0, 131.33, 0.0, -25.0, -133.0, 0.0)

POS_INTERMEDIATE = ArmPosition(-16.41, 28.0, 0.0, 16.41, -28.0, 0.0)
POS_GROUND_INTERMEDIATE = ArmPosition(-8.06, 42.0, 0.0, 8.06, -42.0, 0.0)
POS_HIGH_CONE_EXTENSION_INTERMEDIATE = ArmPosition(0, 90, 0.0, 0, -90, 0.0)
POS_HIGH_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-12, 78.9, 0.0, 12, -74.1, 0.0)
POS_HIGH_CUBE_EXTENSION_INTERMEDIATE = ArmPosition(-8.0, 100.0, 0.0, 8.0, -100.0, 0.0)
# POS_HIGH_CUBE_EXTENSION_INTERMEDIATE = ArmPosition(-8, 100, 8, -100)  #AUTO POS MAYBE
POS_HIGH_CUBE_RETRACTION_INTERMEDIATE = ArmPosition(-8.0, 90.0, 0.0, 8.0, -90.0, 0.0)
# POS_MID_CONE_EXTENSION_INTERMEDIATE = ArmPosition(14.06, 93, -14.06, -93) - without piston extension
# POS_MID_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-8.41, 104.3, 8, -99.8) - without piston extension
POS_MID_CONE_EXTENSION_INTERMEDIATE = ArmPosition(-11.0, 81.73, 0.0, 6.0, -81.73, 0.0)
POS_MID_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-11.0, 81.73, 0.0, 6.0, -81.73, 0.0)
POS_MID_CUBE_INTERMEDIATE = ArmPosition(-8.0, 68.88, 0.0, 8.0, -68.88, 0.0)
POS_SPORT_MODE = ArmPosition(-57.0, 10.50, 0.0, 57.0, -10.50, 0.0) # Setting this to the same as intermediate for right now


def mirror_position(position: ArmPosition) -> ArmPosition:
    """
    Provides rear position angle lookup
    """
    return ArmPosition(position.base_position_rear, position.upper_position_rear, position.wrist_position_rear)
