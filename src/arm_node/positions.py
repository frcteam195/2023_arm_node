from dataclasses import dataclass


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
POS_HIGH_CUBE = ArmPosition(0.029, -0.199)
POS_MID_CONE = ArmPosition(-0.025, -0.255)
POS_HIGH_CONE = ArmPosition(0.046, -0.0139) # may exceed limits

POS_INTERMEDIATE = ArmPosition(-0.068, -0.420)
