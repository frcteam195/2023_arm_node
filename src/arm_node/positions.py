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
    base_position_rear: float = 0.0
    upper_position_rear: float = 0.0


POS_HOME = ArmPosition(0.0, 0.0)

POS_STEAL = ArmPosition(16.70, 82.08, -16.70, -82.08)  # Fake

POS_GROUND_CUBE = ArmPosition(5.06, 33.0, -5.06, -33.0)
POS_GROUND_CONE = ArmPosition(-3.55, 32.16, 4.55, -30.16)
# POS_GROUND_DEAD_CONE = ArmPosition(1, 30.36, -1, -30.36)
POS_PRE_GROUND_DEAD_CONE = ArmPosition(21.38, 40.68, -21.38, -42.0)
POS_GROUND_DEAD_CONE = ArmPosition(40, 33, -40, -36)

# POS_SIDEWAYS_DEAD_CONE = ArmPosition(15.38, 60.68, -15.38, -60.0)
POS_SIDEWAYS_DEAD_CONE = ArmPosition(6.5, 32.0, -6.5, -32.0)


POS_SHELF_CUBE = ArmPosition(-14.98, 77.9, 17.98, -74.1)
POS_SHELF_CONE = ArmPosition(-14.98, 77.9, 17.98, -74.1)

POS_LOW_SCORE = ArmPosition(21.38, 40.68, -21.38, -40.68)
POS_MID_CUBE = ArmPosition(3, 68.88, 0, -68.88)
POS_HIGH_CUBE = ArmPosition(19, 114, -16, -111)
# POS_HIGH_CUBE_AUTO = ArmPosition(8, 100, -8, -100)    #AUTO POS MAYBE
# POS_MID_CONE = ArmPosition(14.06, 104.3, -14.06, -99.8) - without piston extensions
POS_MID_CONE = ArmPosition(-6.5, 81.73, 7, -81.73)
POS_HIGH_CONE = ArmPosition(21, 132.33, -21, -130.5)

POS_INTERMEDIATE = ArmPosition(-16.41, 28.0, 16.41, -28.0)
POS_GROUND_INTERMEDIATE = ArmPosition(-8.06, 42.0, 8.06, -42.0)
POS_HIGH_CONE_EXTENSION_INTERMEDIATE = ArmPosition(0, 90, 0, -90)
POS_HIGH_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-12, 78.9, 12, -74.1)
POS_HIGH_CUBE_EXTENSION_INTERMEDIATE = ArmPosition(-8, 100, 8, -100)
# POS_HIGH_CUBE_EXTENSION_INTERMEDIATE = ArmPosition(-8, 100, 8, -100)  #AUTO POS MAYBE
POS_HIGH_CUBE_RETRACTION_INTERMEDIATE = ArmPosition(-8, 90, 8, -90)
# POS_MID_CONE_EXTENSION_INTERMEDIATE = ArmPosition(14.06, 93, -14.06, -93) - without piston extension
# POS_MID_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-8.41, 104.3, 8, -99.8) - without piston extension
POS_MID_CONE_EXTENSION_INTERMEDIATE = ArmPosition(-11, 81.73, 6, -81.73)
POS_MID_CONE_RETRACTION_INTERMEDIATE = ArmPosition(-11, 81.73, 6, -81.73)
POS_MID_CUBE_INTERMEDIATE = ArmPosition(-8, 68.88, 8, -68.88)
POS_SPORT_MODE = ArmPosition(-57.0, 10.50, 57.0, -10.50) # Setting this to the same as intermediate for right now


def mirror_position(position: ArmPosition) -> ArmPosition:
    """
    Provides rear position angle lookup
    """
    return ArmPosition(position.base_position_rear, position.upper_position_rear)


def rotation_to_angle(rotation: ArmPosition) -> ArmPosition:
    """
    Returns the arm position in degrees based on the provided rotation.
    """
    position = ArmPosition()
    position.base_position = rotation.base_position * 360.0
    position.upper_position = rotation.upper_position * 360.0
    return position
