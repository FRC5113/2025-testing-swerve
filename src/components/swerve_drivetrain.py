from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve, units, utils, hardware
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration,TalonFXConfigurator
from phoenix6.swerve import SwerveModuleConstants, SwerveDrivetrainConstants
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController,RobotBase
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
from lemonlib.smart import SmartNT


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
    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    telemetry: Telemetry

    drivetrain = None

    request = swerve.requests.Idle()
    has_desired_pose = will_reset_to(False)

    x_speed = will_reset_to(0)
    y_speed = will_reset_to(0)
    rot_speed = will_reset_to(0)

    # x_speed = 0
    # y_speed = 0
    # rot_speed = 0

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
        self.smart_nt = SmartNT("SwerveController")

        self.pigeon = self.drivetrain.pigeon2
        SmartDashboard.putData("Pigeon", self.pigeon)
        self.drivetrain.set_operator_perspective_forward(self.pigeon.getRotation2d())

    def get_telemetry(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        return self.drivetrain.get_state()

    def on_enable(self):
        if RobotBase.isSimulation():
            self.drivetrain.tare_everything()
        self.x_controller = self.translation_profile.create_wpi_pid_controller()
        self.y_controller = self.translation_profile.create_wpi_pid_controller()

        self.drive_controller = self.drive_profile.create_ctre_flywheel_controller()
        self.steer_controller = self.steer_profile.create_ctre_turret_controller()

        self.rotation_controller = self.rotation_profile.create_wpi_profiled_pid_controller_radians()

        self.rotation_controller.enableContinuousInput(-math.pi, math.pi)

        drive_config = TalonFXConfiguration().with_slot0(self.drive_controller)
        steer_config = TalonFXConfiguration().with_slot0(self.steer_controller)

        for i in range(4):
            module = self.drivetrain.get_module(i)
            drive = module.drive_motor
            steer = module.steer_motor

            drive.configurator.apply(drive_config)
            steer.configurator.apply(steer_config)

            SmartDashboard.putData(f"Drive/Module{i}", drive)
            SmartDashboard.putData(f"Steer/Module{i}", steer)

            # self.smart_nt.put_number(f"Drive/Module{i}/Error", drive.get_closed_loop_error().value)
            # self.smart_nt.put_number(f"Drive/Module{i}/Setpoint", drive.get_closed_loop_output().value)
            # self.smart_nt.put_number(f"Drive/Module{i}/Position", drive.get_closed_loop_reference().value)
            # self.smart_nt.put_number(f"Drive/Module{i}/FeedForward", drive.get_closed_loop_feed_forward().value)

            # self.smart_nt.put_number(f"Steer/Module{i}/Error", steer.get_closed_loop_error().value)
            # self.smart_nt.put_number(f"Steer/Module{i}/Setpoint", steer.get_closed_loop_output().value)
            # self.smart_nt.put_number(f"Steer/Module{i}/Position", steer.get_closed_loop_reference().value)
            # self.smart_nt.put_number(f"Steer/Module{i}/FeedForward", steer.get_closed_loop_feed_forward().value)

        self.holonomic_controller = HolonomicDriveController(
            self.x_controller, self.y_controller, self.rotation_controller
        )

    def set_forward(self):
        self.drivetrain.set_operator_perspective_forward(self.pigeon.getRotation2d())


    def set_desired_pose(self, pose: Pose2d):
        self.desired_pose = pose
        self.has_desired_pose = True

    def drive_robot_centric(
        self,
        x: units.meters_per_second,
        y: units.meters_per_second,
        rotation: units.radians_per_second,
    ):
        self.x_speed = x
        self.y_speed = y
        self.rot_speed = rotation
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
        self.x_speed = x
        self.y_speed = y
        self.rot_speed = rotation
        self.request = (
            swerve.requests.FieldCentric()
            .with_velocity_x(self.x_speed)
            .with_velocity_y(self.y_speed)
            .with_rotational_rate(self.rot_speed)
        )
        print(self.request.velocity_x, self.request.velocity_y, self.request.rotational_rate)

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
            math.sqrt(math.pow(sample.vx, 2) + math.pow(sample.vy, 2)),
            sample.heading,
        )
        speeds = ChassisSpeeds(
            sample.vx + holospeeds.vx,
            sample.vy + holospeeds.vy,
            sample.omega + holospeeds.omega,
        )
        print(holospeeds)
        self.request = (
            swerve.requests.RobotCentric()
            .with_velocity_x(sample.vx)
            .with_velocity_y(sample.vy)
            .with_rotational_rate(sample.omega)
        )

    def execute(self):
        # print(self.drive_control)
        self.drivetrain.set_control(self.request)
        
        self.telemetry.telemeterize(self.drivetrain.get_state())

