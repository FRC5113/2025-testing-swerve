from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve, units, utils, hardware
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.swerve import SwerveModuleConstants, SwerveDrivetrainConstants
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d

from lemonlib.util import MagicSysIdRoutine
from lemonlib.smart import SmartProfile

from .telemetry import Telemetry
from magicbot import will_reset_to

from typing import Optional, Union, List

class SwerveDrive:

    

    max_speed: float
    drive_profile: SmartProfile
    steer_profile: SmartProfile
    drive_gear_ratio: float
    steer_gear_ratio: float
    couple_ratio: float
    wheel_radius: units.meter
    steer_closed_loop_output: swerve.ClosedLoopOutputType
    drive_closed_loop_output: swerve.ClosedLoopOutputType
    slip_current: units.ampere
    speed_at_12_volts: units.meters_per_second
    drive_motor_type: swerve.DriveMotorArrangement
    steer_motor_type: swerve.SteerMotorArrangement
    steer_feedback_type: swerve.SteerFeedbackType
    drive_initial_configs: TalonFXConfiguration
    steer_initial_configs: TalonFXConfiguration
    encoder_initial_configs: CANcoderConfiguration
    steer_inertia: units.kilogram_square_meter
    drive_inertia: units.kilogram_square_meter
    steer_friction_voltage: units.volt
    drive_friction_voltage: units.volt
    front_left_steer_motor_id: int
    front_left_drive_motor_id: int
    front_left_encoder_id: int
    front_left_encoder_offset: units.rotation

    invert_left_side: bool
    front_right_steer_motor_id: int
    front_right_drive_motor_id: int
    front_right_encoder_id: int
    front_right_encoder_offset: units.rotation

    invert_right_side: bool
    back_left_steer_motor_id: int
    back_left_drive_motor_id: int
    back_left_encoder_id: int
    back_left_encoder_offset: units.rotation
    back_right_steer_motor_id: int
    back_right_drive_motor_id: int
    back_right_encoder_id: int
    back_right_encoder_offset: units.rotation
    drivetrain_constants: SwerveDrivetrainConstants

    offset: units.meter
    steer_motor_inverted: bool
    encoder_inverted: bool

    drivetrain = None

    drive_control = will_reset_to(swerve.requests.Idle())

    SIM_LOOP_PERIOD =  0.005

    def setup(self) -> None:
        self.drive_controller = self.drive_profile.create_ctre_flywheel_controller()
        self.steer_controller = self.steer_profile.create_ctre_turret_controller()

        self.constants_creator: swerve.SwerveModuleConstantsFactory[
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration,
        ] = (
            swerve.SwerveModuleConstantsFactory()
            .with_drive_motor_gear_ratio(self.drive_gear_ratio)
            .with_steer_motor_gear_ratio(self.steer_gear_ratio)
            .with_coupling_gear_ratio(self.couple_ratio)
            .with_wheel_radius(self.wheel_radius)
            .with_steer_motor_gains(self.steer_controller)
            .with_drive_motor_gains(self.drive_controller)
            .with_steer_motor_closed_loop_output(self.steer_closed_loop_output)
            .with_drive_motor_closed_loop_output(self.drive_closed_loop_output)
            .with_slip_current(self.slip_current)
            .with_speed_at12_volts(self.speed_at_12_volts)
            .with_drive_motor_type(self.drive_motor_type)
            .with_steer_motor_type(self.steer_motor_type)
            .with_feedback_source(self.steer_feedback_type)
            .with_drive_motor_initial_configs(self.drive_initial_configs)
            .with_steer_motor_initial_configs(self.steer_initial_configs)
            .with_encoder_initial_configs(self.encoder_initial_configs)
            .with_steer_inertia(self.steer_inertia)
            .with_drive_inertia(self.drive_inertia)
            .with_steer_friction_voltage(self.steer_friction_voltage)
            .with_drive_friction_voltage(self.drive_friction_voltage)
        )
        self.front_left = self.constants_creator.create_module_constants(
            self.front_left_steer_motor_id,
            self.front_left_drive_motor_id,
            self.front_left_encoder_id,
            self.front_left_encoder_offset,
            self.offset,
            self.offset,
            self.invert_left_side,
            self.steer_motor_inverted,
            self.encoder_inverted,
        )
        self.front_right = self.constants_creator.create_module_constants(
            self.front_right_steer_motor_id,
            self.front_right_drive_motor_id,
            self.front_right_encoder_id,
            self.front_right_encoder_offset,
            self.offset,
            -self.offset,
            self.invert_right_side,
            self.steer_motor_inverted,
            self.encoder_inverted,
        )
        self.back_left = self.constants_creator.create_module_constants(
            self.back_left_steer_motor_id,
            self.back_left_drive_motor_id,
            self.back_left_encoder_id,
            self.back_left_encoder_offset,
            -self.offset,
            self.offset,
            self.invert_left_side,
            self.steer_motor_inverted,
            self.encoder_inverted,
        )
        self.back_right = self.constants_creator.create_module_constants(
            self.back_right_steer_motor_id,
            self.back_right_drive_motor_id,
            self.back_right_encoder_id,
            self.back_right_encoder_offset,
            self.offset,
            -self.offset,
            self.invert_right_side,
            self.steer_motor_inverted,
            self.encoder_inverted,
        )
        self.drivetrain =  swerve.SwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            self.drivetrain_constants,
            [
                self.front_left,
                self.front_right,
                self.back_left,
                self.back_right,
            ],
        )


        self.telemetry = Telemetry(self.max_speed)
        self.drivetrain.register_telemetry(lambda state: self.telemetry.telemeterize(state))

        self.sim_notifier: Notifier | None = None
        self.last_sim_time: units.second = 0.0

        if utils.is_simulation():
            self.start_sim_thread()

    def drive_robot_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.drive_control = swerve.requests.RobotCentric().with_velocity_x(x).with_velocity_y(y).with_rotational_rate(rotation)


    def drive_field_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.drive_control = swerve.requests.FieldCentric()
        self.drive_control.velocity_x = x
        self.drive_control.velocity_y = y
        self.drive_control.rotational_rate = rotation
        
    def start_sim_thread(self):
        def sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self.last_sim_time
            self.last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.drivetrain.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self.last_sim_time = utils.get_current_time_seconds()
        self.sim_notifier = Notifier(sim_periodic)
        self.sim_notifier.startPeriodic(self.SIM_LOOP_PERIOD)


    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: units.second,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        self.drivetrain.add_vision_measurement(
            self,
            vision_robot_pose,
            utils.fpga_to_current_time(timestamp),
            vision_measurement_std_devs,
        )

    def execute(self):
        print(self.drive_control)
        self.drivetrain.set_control(self.drive_control)
