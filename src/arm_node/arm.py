from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from arm_node.positions import ArmPosition


class Arm:
    def __init__(self, baseMotor, upperMotor, baseBrake, upperBrake, extension):
        self.baseMotor: Motor = baseMotor
        self.upperMotor: Motor = upperMotor
        self.baseBrake: Solenoid = baseBrake
        self.upperBrake: Solenoid = upperBrake
        self.extension: Solenoid = extension

    def set_motion_magic(self, position: ArmPosition):
        self.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
        self.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)

    def set_percent_output(self, base=0.0, upper=0.0):
        self.baseMotor.set(ControlMode.PERCENT_OUTPUT, base)
        self.upperMotor.set(ControlMode.PERCENT_OUTPUT, upper)

    def is_at_setpoint(self, base_tolerance, upper_tolerance) -> bool:
        base_in_range = self.baseMotor.is_at_setpoint(base_tolerance)
        upper_in_range = self.upperMotor.is_at_setpoint(upper_tolerance)
        return base_in_range and upper_in_range

    def enable_brakes(self):
        self.baseBrake.set(SolenoidState.OFF)
        self.upperBrake.set(SolenoidState.OFF)

    def disable_brakes(self):
        self.baseBrake.set(SolenoidState.ON)
        self.upperBrake.set(SolenoidState.ON)

    def extend(self):
        self.extension.set(SolenoidState.ON)

    def retract(self):
        self.extension.set(SolenoidState.OFF)
