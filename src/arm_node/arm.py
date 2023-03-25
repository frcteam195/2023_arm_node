from arm_node.positions import ArmPosition

from ck_ros_msgs_node.msg import Arm_Status

from actions_node.game_specific_actions.constant import RollerState

from ck_utilities_py_node.ckmath import limit
from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

class Arm:
    """
    Arm subsystem.
    """
    def __init__(self, baseMotor, upperMotor, wristMotor, intakeMotor, baseBrake, upperBrake, extension, home_position, lower_limits, upper_limits):
        self.baseMotor: Motor = baseMotor
        self.upperMotor: Motor = upperMotor
        self.wristMotor: Motor = wristMotor
        self.intakeMotor: Motor = intakeMotor
        self.baseBrake: Solenoid = baseBrake
        self.upperBrake: Solenoid = upperBrake
        self.extension: Solenoid = extension

        self.home_position: ArmPosition = home_position
        self.lower_limits: ArmPosition = lower_limits
        self.upper_limits: ArmPosition = upper_limits

        self.intake_goal: RollerState = RollerState.Off

        self.__upper_arm_default_cruise_vel = self.upperMotor.config.motionCruiseVelocity
        self.__upper_arm_default_accel = self.upperMotor.config.motionCruiseAcceleration
        self.__upper_arm_default_s_curve = self.upperMotor.config.motionSCurveStrength

        self.__lower_arm_default_cruise_vel = self.baseMotor.config.motionCruiseVelocity
        self.__lower_arm_default_accel = self.baseMotor.config.motionCruiseAcceleration
        self.__lower_arm_default_s_curve = self.baseMotor.config.motionSCurveStrength

    def set_motion_magic(self, angle: ArmPosition):
        """
        Converts the arm positions from degrees to rotations, then commands motion magic.
        """
        self.set_motion_magic_raw(self.__angle_to_rotation(angle))

    def set_motion_magic_raw(self, position: ArmPosition):
        """
        Sets the motion magic command with rotations.
        """
        self.baseMotor.set(ControlMode.MOTION_MAGIC, position.base_position)
        self.upperMotor.set(ControlMode.MOTION_MAGIC, position.upper_position)
        self.wristMotor.set(ControlMode.MOTION_MAGIC, position.wrist_position)

    def set_percent_output(self, base=0.0, upper=0.0, wrist=0.0):
        """
        Sets the various arm motors to percent output mode.
        """
        self.baseMotor.set(ControlMode.PERCENT_OUTPUT, base)
        self.upperMotor.set(ControlMode.PERCENT_OUTPUT, upper)
        self.wristMotor.set(ControlMode.PERCENT_OUTPUT, wrist)

    def is_at_setpoint(self, base_tolerance, upper_tolerance, wrist_tolerance) -> bool:
        """
        Checks if the arm and wrist are at the setpoint, using tolerances in degrees.
        """
        return self.is_at_setpoint_raw(base_tolerance / 360.0, upper_tolerance / 360.0, wrist_tolerance / 360.0)

    def is_at_setpoint_raw(self, base_tolerance, upper_tolerance, wrist_tolerance) -> bool:
        """
        Checks if the arm and wrist are at the setpoint, using tolerances in rotations.
        """
        base_in_range = self.baseMotor.is_at_setpoint(base_tolerance)
        upper_in_range = self.upperMotor.is_at_setpoint(upper_tolerance)
        wrist_in_range = self.wristMotor.is_at_setpoint(wrist_tolerance)
        return base_in_range and upper_in_range and wrist_in_range

    def get_base_deviation_deg(self) -> float:
        """
        Gets the deviation of the base arm in degrees.
        """
        return (self.baseMotor.get_setpoint() - self.baseMotor.get_sensor_position()) * 360.0

    def get_upper_deviation_deg(self) -> float:
        """
        Gets the deviation of the upper arm in degrees.
        """
        return (self.upperMotor.get_setpoint() - self.upperMotor.get_sensor_position()) * 360.0

    def get_wrist_deviation_deg(self) -> float:
        """
        Gets the deviation of the wrist in degrees.
        """
        return (self.wristMotor.get_setpoint() - self.wristMotor.get_sensor_position()) * 360.0

    def config_arm_fast(self):
        self.upperMotor.config.motionSCurveStrength = 0
        self.upperMotor.config.motionCruiseAcceleration = self.__upper_arm_default_accel * 1.2
        self.upperMotor.config.motionCruiseVelocity = self.__upper_arm_default_cruise_vel * 1.2
        self.upperMotor.apply()

    def config_arm_normal(self):
        self.upperMotor.config.motionSCurveStrength = self.__upper_arm_default_s_curve
        self.upperMotor.config.motionCruiseAcceleration = self.__upper_arm_default_accel
        self.upperMotor.config.motionCruiseVelocity = self.__upper_arm_default_cruise_vel
        self.upperMotor.apply()

    def config_arm_slow(self):
        self.upperMotor.config.motionSCurveStrength = 3
        self.upperMotor.config.motionCruiseAcceleration = self.__upper_arm_default_accel * 0.8
        self.upperMotor.config.motionCruiseVelocity = self.__upper_arm_default_cruise_vel * 1
        self.upperMotor.apply()

    def config_lower_arm_fast(self):
        self.baseMotor.config.motionSCurveStrength = 1
        self.baseMotor.config.motionCruiseAcceleration = self.__lower_arm_default_accel * 1.4
        self.baseMotor.config.motionCruiseVelocity = self.__lower_arm_default_cruise_vel * 1.3
        self.baseMotor.apply()

    def config_lower_arm_normal(self):
        self.baseMotor.config.motionSCurveStrength = self.__lower_arm_default_s_curve
        self.baseMotor.config.motionCruiseAcceleration = self.__lower_arm_default_accel
        self.baseMotor.config.motionCruiseVelocity = self.__lower_arm_default_cruise_vel
        self.baseMotor.apply()

    def config_lower_arm_slow(self):
        self.baseMotor.config.motionSCurveStrength = 7
        self.baseMotor.config.motionCruiseAcceleration = self.__lower_arm_default_accel
        self.baseMotor.config.motionCruiseVelocity = self.__lower_arm_default_cruise_vel
        self.baseMotor.apply()

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

    def control_intake(self, state: RollerState, speed: float = 1.0):
        """
        Controls the intake rollers. Default speed is full.
        """
        speed = limit(abs(speed), 0.0, 1.0)

        if state in (RollerState.Intake_Cone, RollerState.Outtake_Cube):
            self.intakeMotor.set(ControlMode.PERCENT_OUTPUT, speed)
        elif state in (RollerState.Intake_Cube, RollerState.Outtake_Cone):
            self.intakeMotor.set(ControlMode.PERCENT_OUTPUT, -speed)
        else:
            self.intakeMotor.set(ControlMode.PERCENT_OUTPUT, 0.0)

    def is_retracted(self) -> bool:
        return self.wristMotor.get_reverse_limit_closed()

    def get_angle(self) -> ArmPosition:
        """
        Gets the position of the arm in degrees.
        """
        position = self.get_raw_position()

        position.base_position = (position.base_position - self.home_position.base_position) * 360.0
        position.upper_position = (position.upper_position - self.home_position.upper_position) * 360.0
        position.wrist_position = (position.wrist_position - self.home_position.wrist_position) * 360.0

        return position

    def get_raw_position(self) -> ArmPosition:
        """
        Gets the raw position of the arm.
        """
        position = ArmPosition()

        position.base_position = self.baseMotor.get_sensor_position()
        position.upper_position = self.upperMotor.get_sensor_position()
        position.wrist_position = self.wristMotor.get_sensor_position()

        return position

    def get_angular_velocity(self) -> ArmPosition:
        """
        Gets the angular velocity of the arm and wrist.
        """
        velocity = self.get_raw_velocity()

        velocity.base_position = velocity.base_position * 360.0
        velocity.upper_position = velocity.upper_position * 360.0
        velocity.wrist_position = velocity.wrist_position * 360.0

        return velocity

    def get_raw_velocity(self) -> ArmPosition:
        """
        Gets the raw velocity of the arm and wrist.
        """
        velocity = ArmPosition()

        velocity.base_position = self.baseMotor.get_sensor_velocity()
        velocity.upper_position = self.upperMotor.get_sensor_velocity()
        velocity.wrist_position = self.wristMotor.get_sensor_velocity()

        return velocity

    def __angle_to_rotation(self, angle: ArmPosition) -> ArmPosition:
        """
        Returns the arm position in rotation based on the given angles from each home value.
        """
        position = ArmPosition()

        position.base_position = angle.base_position / 360.0 + self.home_position.base_position
        position.upper_position = angle.upper_position / 360.0 + self.home_position.upper_position
        position.wrist_position = angle.wrist_position / 360.0 + self.home_position.wrist_position

        return position

    def get_status(self) -> Arm_Status:
        """
        Returns a fully populated status message of the arm.
        """
        master_sticky_faults = self.baseMotor.get_sticky_faults()
        follower_sticky_faults = self.baseMotor.get_sticky_faults()

        raw_position = self.get_raw_position()
        angle = self.get_angle()

        raw_velocity = self.get_raw_velocity()
        angular_velocity = self.get_angular_velocity()

        status_message = Arm_Status()

        status_message.arm_base_actual_position = raw_position.base_position
        status_message.arm_upper_actual_position = raw_position.upper_position
        status_message.arm_wrist_actual_position = raw_position.wrist_position

        status_message.arm_base_angle = angle.base_position
        status_message.arm_upper_angle = angle.upper_position
        status_message.arm_wrist_angle = angle.wrist_position

        status_message.arm_base_velocity = raw_velocity.base_position
        status_message.arm_upper_velocity = raw_velocity.upper_position
        status_message.arm_wrist_velocity = raw_velocity.wrist_position

        status_message.arm_base_angular_velocity = angular_velocity.base_position
        status_message.arm_upper_angular_velocity = angular_velocity.upper_position
        status_message.arm_wrist_angular_velocity = angular_velocity.wrist_position

        status_message.extended = not self.is_retracted()
        status_message.left_arm_base_remote_loss_of_signal = master_sticky_faults.RemoteLossOfSignal
        status_message.right_arm_base_remote_loss_of_signal = follower_sticky_faults.RemoteLossOfSignal
        status_message.left_arm_base_reset_during_en = master_sticky_faults.ResetDuringEn
        status_message.right_arm_base_reset_during_en = follower_sticky_faults.ResetDuringEn
        status_message.left_arm_base_hardware_ESD_reset = master_sticky_faults.HardwareESDReset
        status_message.right_arm_base_hardware_ESD_reset = follower_sticky_faults.HardwareESDReset
        status_message.left_arm_base_supply_unstable = master_sticky_faults.SupplyUnstable
        status_message.right_arm_base_supply_unstable = follower_sticky_faults.SupplyUnstable

        return status_message
