from dataclasses import dataclass
import rospy


ALLOWED_DEVIATION_PCT = 0.10
BASE_ALLOWED_DEVIATION = abs(rospy.get_param("/arm_node/baseArmMaster_forwardSoftLimit") - rospy.get_param("/arm_node/baseArmMaster_reverseSoftLimit")) * ALLOWED_DEVIATION_PCT
UPPER_ALLOWED_DEVIATION = abs(rospy.get_param("/arm_node/upperArmMaster_forwardSoftLimit") - rospy.get_param("/arm_node/upperArmMaster_reverseSoftLimit")) * ALLOWED_DEVIATION_PCT

@dataclass
class ArmPosition:
    base_position: float
    upper_position: float


POS_HOME = ArmPosition(-0.022402006267613173, -0.5)

POS_GROUND_CUBE = ArmPosition(-0.0193, -0.427)
POS_GROUND_CONE = ArmPosition(-0.0267, -0.419)
POS_GROUND_DEAD_CODE = ArmPosition(0.013, -0.407)

POS_SHELF = ArmPosition(-0.064, -0.285)

POS_LOW_SCORE = ArmPosition(0.037, -0.387)
POS_MID_CUBE = ArmPosition(0.023, -0.272)
POS_HIGH_CUBE = ArmPosition(0.029, -0.175)
# POS_HIGH_CUBE = ArmPosition(0.029, -0.199)
POS_MID_CONE = ArmPosition(-0.025, -0.255)
POS_HIGH_CONE = ArmPosition(0.046, -0.0139) # may exceed limits

POS_INTERMEDIATE = ArmPosition(-0.068, -0.420)


def mirror_position(position: ArmPosition):
    mirrored_base = POS_HOME.base_position - (position.base_position - POS_HOME.base_position)
    mirrored_upper = POS_HOME.upper_position - (position.upper_position - POS_HOME.upper_position)
    return ArmPosition(mirrored_base, mirrored_upper)
