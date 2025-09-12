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
from wpilib import Field2d, SmartDashboard
from choreo.trajectory import SwerveSample
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.controller import HolonomicDriveController


class SwerveDrive:
    max_speed: units.meters_per_second
    max_angular_rate: units.radians_per_second
    drivetrain_constants: SwerveDrivetrainConstants
    front_left: SwerveModuleConstants
    front_right: SwerveModuleConstants
    back_left: SwerveModuleConstants
    back_right: SwerveModuleConstants

    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    telemetry: Telemetry

    drivetrain = None

    request = swerve.requests.Idle()
    has_desired_pose = will_reset_to(False)

    state = swerve.SwerveDrivetrain.SwerveDriveState()

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
        self.desired_pose = Pose2d()

    def get_telemetry(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        return self.drivetrain.get_state()

    def update_configs(self):
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

    def on_enable(self):
        self.x_controller = self.translation_profile.create_wpi_pid_controller()
        self.y_controller = self.translation_profile.create_wpi_pid_controller()
        self.theta_controller = (
            self.rotation_profile.create_wpi_profiled_pid_controller_radians()
        )
        self.holonomic_controller = HolonomicDriveController(
            self.x_controller, self.y_controller, self.theta_controller
        )
        self.theta_controller.enableContinuousInput(-math.pi, math.pi)



    def set_desired_pose(self, pose: Pose2d):
        self.desired_pose = pose
        self.has_desired_pose = True

    def drive_robot_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.request = (
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
        self.request = (
            swerve.requests.FieldCentric()
            .with_velocity_x(x * self.max_speed)
            .with_velocity_y(y * self.max_speed)
            .with_rotational_rate(rotation * self.max_speed)
        )

    def sim_update(self, delta_time: units.second, battery_voltage: units.volt):
        self.drivetrain.update_sim_state(delta_time, battery_voltage)

    def apply_request(self, request: swerve.requests.SwerveRequest):
        self.request = request

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

    def get_velocity(self) -> ChassisSpeeds:
        return self.state.speeds

    def follow_trajectory(self, sample: SwerveSample):
        holospeeds = self.holonomic_controller.calculate(
            self.state.pose,
            sample.get_pose(),
            0.0,
            sample.get_pose().rotation(),
        )
        speeds = ChassisSpeeds(
            sample.vx + holospeeds.vx,
            sample.vy + holospeeds.vy,
            sample.omega + holospeeds.omega,
        )
        self.request = (
            swerve.requests.RobotCentric()
            .with_velocity_x(speeds.vx)
            .with_velocity_y(speeds.vy)
            .with_rotational_rate(speeds.omega)
        )

    def execute(self):
        # print(self.drive_control)
        self.drivetrain.set_control(self.request)
        self.telemetry.telemeterize(self.drivetrain.get_state())

