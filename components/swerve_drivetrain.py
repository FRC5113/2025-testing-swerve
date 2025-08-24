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
    max_speed: units.meters_per_second
    drivetrain_constants: SwerveDrivetrainConstants
    front_left: SwerveModuleConstants
    front_right: SwerveModuleConstants
    back_left: SwerveModuleConstants
    back_right: SwerveModuleConstants

    drive_profile: SmartProfile
    steer_profile: SmartProfile

    drivetrain = None

    drive_control = will_reset_to(swerve.requests.Idle())

    SIM_LOOP_PERIOD = 0.005

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
        self.drive_control = (
            swerve.requests.RobotCentric()
            .with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(rotation)
        )

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
            self.drivetrain.update_sim_state(
                delta_time, RobotController.getBatteryVoltage()
            )

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
