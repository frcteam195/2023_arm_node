"""
Defines all the arm positions in degrees.
"""

from dataclasses import dataclass
import rospy

ALLOWED_DEVIATION_PCT = 0.05
BASE_ALLOWED_DEVIATION = abs(rospy.get_param("/arm_node/baseArmMaster_forwardSoftLimit") -
                             rospy.get_param("/arm_node/baseArmMaster_reverseSoftLimit")) * ALLOWED_DEVIATION_PCT
UPPER_ALLOWED_DEVIATION = abs(rospy.get_param("/arm_node/upperArmMaster_forwardSoftLimit") -
                              rospy.get_param("/arm_node/upperArmMaster_reverseSoftLimit")) * ALLOWED_DEVIATION_PCT


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

POS_GROUND_CUBE = ArmPosition(5.06, 33.0, -8.06, -33.0)
POS_GROUND_CONE = ArmPosition(-3.55, 32.16, 1.55, -29.16)
POS_GROUND_DEAD_CONE = ArmPosition(12.74, 27.36, -12.74, -27.36)

POS_SHELF_CUBE = ArmPosition(-14.98, 78.9, 14.98, -74.1)
POS_SHELF_CONE = ArmPosition(-14.98, 78.9, 14.98, -74.1)

POS_LOW_SCORE = ArmPosition(21.38, 40.68, -21.38, -40.68)
POS_MID_CUBE = ArmPosition(16.34, 83.88, -18.34, -83.88)
POS_HIGH_CUBE = ArmPosition(21.5, 116.5, -23.5, -116.0)
POS_MID_CONE = ArmPosition(14.06, 103.3, -14.06, -98.8)
POS_HIGH_CONE = ArmPosition(19.74, 131.33, -25, -133.88)

POS_INTERMEDIATE = ArmPosition(-16.41, 28.0, 16.41, -28.0)
POS_GROUND_INTERMEDIATE = ArmPosition(8.06, 58.0, -8.06, -58.0)
POS_HIGH_INTERMEDIATE = ArmPosition(-16.41, 90.0, 16.41, -90.0)


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
