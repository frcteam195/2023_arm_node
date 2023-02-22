from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from arm_node.positions import ArmPosition

from ck_ros_msgs_node.msg import Arm_Status

from actions_node.game_specific_actions.constant import WristPosition


class Arm:
    def __init__(self, baseMotor, upperMotor, wristMotor, baseBrake, upperBrake, extension, home_position, lower_limits, upper_limits):
        self.baseMotor: Motor = baseMotor
        self.upperMotor: Motor = upperMotor
        self.wristMotor: Motor = wristMotor
        self.baseBrake: Solenoid = baseBrake
        self.upperBrake: Solenoid = upperBrake
        self.extension: Solenoid = extension
        self.home_position: ArmPosition = home_position
        self.lower_limits: ArmPosition = lower_limits
        self.upper_limits: ArmPosition = upper_limits

        self.wrist_goal: WristPosition = WristPosition.Zero

    def set_motion_magic(self, angle: ArmPosition):
        self.set_motion_magic_raw(self.__angle_to_rotation(angle))

    def set_motion_magic_raw(self, position: ArmPosition):
        self.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
        self.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)

    def set_percent_output(self, base=0.0, upper=0.0):
        self.baseMotor.set(ControlMode.PERCENT_OUTPUT, base)
        self.upperMotor.set(ControlMode.PERCENT_OUTPUT, upper)

    def is_at_setpoint(self, base_tolerance, upper_tolerance) -> bool:
        return self.is_at_setpoint_raw(base_tolerance / 360.0, upper_tolerance / 360.0)

    def is_at_setpoint_raw(self, base_tolerance, upper_tolerance) -> bool:
        base_in_range = self.baseMotor.is_at_setpoint(base_tolerance)
        upper_in_range = self.upperMotor.is_at_setpoint(upper_tolerance)
        return base_in_range and upper_in_range

    def stow_wrist(self):
        if self.wrist_goal == WristPosition.Left_90:
            self.wrist_goal = WristPosition.Zero

        if self.wrist_goal == WristPosition.Zero:
            self.wristMotor.set(ControlMode.MOTION_MAGIC, WristPosition.Zero.value)
        elif self.wrist_goal == WristPosition.Left_180:
            self.wristMotor.set(ControlMode.MOTION_MAGIC, WristPosition.Left_180.value)

    def set_wrist(self, position: WristPosition):
        self.wristMotor.set(ControlMode.MOTION_MAGIC, position.value)

    def limp_wrist(self, position: WristPosition):
        self.wristMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

    def wrist_at_setpoint(self, tolerance: float) -> bool:
        return self.wristMotor.is_at_setpoint(tolerance)

    def get_wrist_goal(self) -> WristPosition:
        return self.wrist_goal

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

    def get_angle(self) -> ArmPosition:
        position = self.get_raw_position()
        position.base_position = (position.base_position - self.home_position.base_position) * 360.0
        position.upper_position = (position.upper_position - self.home_position.upper_position) * 360.0

        return position

    def get_raw_position(self) -> ArmPosition:
        position = ArmPosition()
        position.base_position = self.baseMotor.get_sensor_position()
        position.upper_position = self.upperMotor.get_sensor_position()

        return position

    def get_angular_velocity(self) -> ArmPosition:
        velocity = self.get_raw_velocity()
        velocity.base_position = velocity.base_position * 360.0
        velocity.upper_position = velocity.upper_position * 360.0

        return velocity

    def get_raw_velocity(self) -> ArmPosition:
        velocity = ArmPosition()
        velocity.base_position = self.baseMotor.get_sensor_velocity()
        velocity.upper_position = self.baseMotor.get_sensor_velocity()

        return velocity

    def get_wrist_angle(self) -> float:
        return self.get_raw_wrist_position() * 360.0

    def get_raw_wrist_position(self) -> float:
        return self.wristMotor.get_sensor_position()

    def get_wrist_angular_velocity(self) -> float:
        return self.get_raw_wrist_velocity() * 360.0

    def get_raw_wrist_velocity(self) -> float:
        return self.wristMotor.get_sensor_velocity()

    def __angle_to_rotation(self, angle: ArmPosition) -> ArmPosition:
        """
        Returns the arm position in rotation based on the given angles from each home value
        """
        new_pos = ArmPosition()
        new_pos.base_position = angle.base_position / 360.0 + self.home_position.base_position
        new_pos.upper_position = angle.upper_position / 360.0 + self.home_position.upper_position

        return new_pos

    def get_status(self) -> Arm_Status:
        master_sticky_faults = self.baseMotor.get_sticky_faults()
        follower_sticky_faults = self.baseMotor.get_sticky_faults()

        raw_position = self.get_raw_position()
        angle = self.get_angle()

        raw_velocity = self.get_raw_velocity()
        angular_velocity = self.get_angular_velocity()

        raw_wrist = self.get_raw_wrist_position()
        wrist_angle = self.get_wrist_angle()

        raw_wrist_velocity = self.get_raw_wrist_velocity()
        wrist_angular_velocity = self.get_wrist_angular_velocity()

        status_message = Arm_Status()

        status_message.arm_base_actual_position = raw_position.base_position
        status_message.arm_upper_actual_position = raw_position.upper_position
        # status_message.arm_wrist_actual_position = wristMotor.get_sensor_position()
        status_message.arm_wrist_actual_position = raw_wrist

        status_message.arm_base_angle = angle.base_position
        status_message.arm_upper_angle = angle.upper_position
        status_message.arm_wrist_angle = wrist_angle

        status_message.arm_base_velocity = raw_velocity.base_position
        status_message.arm_upper_velocity = raw_velocity.upper_position
        status_message.arm_wrist_velocity = raw_wrist_velocity

        status_message.arm_base_angular_velocity = angular_velocity.base_position
        status_message.arm_upper_angular_velocity = angular_velocity.upper_position
        status_message.arm_wrist_angular_velocity = wrist_angular_velocity

        status_message.extended = self.extension.get() == SolenoidState.ON
        status_message.left_arm_base_remote_loss_of_signal = master_sticky_faults.RemoteLossOfSignal
        status_message.right_arm_base_remote_loss_of_signal = follower_sticky_faults.RemoteLossOfSignal
        status_message.left_arm_base_reset_during_en = master_sticky_faults.ResetDuringEn
        status_message.right_arm_base_reset_during_en = follower_sticky_faults.ResetDuringEn
        status_message.left_arm_base_hardware_ESD_reset = master_sticky_faults.HardwareESDReset
        status_message.right_arm_base_hardware_ESD_reset = follower_sticky_faults.HardwareESDReset
        status_message.left_arm_base_supply_unstable = master_sticky_faults.SupplyUnstable
        status_message.right_arm_base_supply_unstable = follower_sticky_faults.SupplyUnstable

        return status_message
