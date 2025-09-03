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

from components.telemetry import Telemetry
from magicbot import will_reset_to

from typing import Optional, Union, List


class SwerveDrive:
    max_speed: units.meters_per_second
    max_angular_rate: units.radians_per_second
    drivetrain_constants: SwerveDrivetrainConstants
    front_left: SwerveModuleConstants
    front_right: SwerveModuleConstants
    back_left: SwerveModuleConstants
    back_right: SwerveModuleConstants

    drive_profile: SmartProfile
    steer_profile: SmartProfile

    drivetrain = None

    drive_control = swerve.requests.Idle()


    def setup(self) -> None:

        self.drivetrain = swerve.SwerveDrivetrain(
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
        self.drivetrain.register_telemetry(
            lambda state: self.telemetry.telemeterize(state)
        )

        self.drivetrain.register_telemetry(
            lambda state: self.telemetry.telemeterize(state)
        )



    def drive_robot_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.drive_control = (
            swerve.requests.RobotCentric()
            .with_velocity_x(x * self.max_speed)
            .with_velocity_y(y * self.max_speed)
            .with_rotational_rate(rotation * self.max_speed)
        )

    def drive_field_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.drive_control = (
            swerve.requests.RobotCentric()
            .with_velocity_x(x * self.max_speed)
            .with_velocity_y(y * self.max_speed)
            .with_rotational_rate(rotation  * self.max_speed)
        )


    def sim_update(self,delta_time: units.second, battery_voltage: units.volt):
        self.drivetrain.update_sim_state(delta_time, battery_voltage)


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
        # print(self.drive_control)
        self.drivetrain.set_control(self.drive_control)
