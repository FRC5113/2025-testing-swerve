from lemonlib import LemonRobot

# from components.telemetry import Telemetry

from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from lemonlib import LemonInput

from lemonlib.smart import SmartPreference, SmartProfile

from phoenix6 import CANBus, configs, hardware, signals, swerve, units

from components.swerve_drivetrain import SwerveDrive
from wpimath.units import inchesToMeters
from wpilib import RobotBase
from lemonlib.smart import SmartProfile


class MyRobot(LemonRobot):

    drivetrain: SwerveDrive

    def createObjects(self):
        """
        SWERVE
        """
        self.max_speed = 4.73
        self.max_angular_rate = rotationsToRadians(0.75)

        self.steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
        self.drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

        self.drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
        self.steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

        self.steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

        self.slip_current: units.ampere = 120.0

        self.drive_initial_configs = configs.TalonFXConfiguration()
        self.steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_stator_current_limit_enable(True)
        )
        self.encoder_initial_configs = configs.CANcoderConfiguration()
        self.pigeon_configs: configs.Pigeon2Configuration | None = None

        self.canbus = CANBus("", "./logs/example.hoot")

        self.speed_at_12_volts: units.meters_per_second = 4.73

        self.couple_ratio = 3.5714285714285716

        self.drive_gear_ratio = 6.746031746031747
        self.steer_gear_ratio = 21.428571428571427
        self.wheel_radius: units.meter = inchesToMeters(2)

        self.invert_left_side = False
        self.invert_right_side = True

        self.pigeon_id = 30

        self.steer_inertia: units.kilogram_square_meter = 0.01
        self.drive_inertia: units.kilogram_square_meter = 0.01
        self.steer_friction_voltage: units.volt = 0.2
        self.drive_friction_voltage: units.volt = 0.2

        self.drivetrain_constants = (
            swerve.SwerveDrivetrainConstants()
            .with_can_bus_name(self.canbus.name)
            .with_pigeon2_id(self.pigeon_id)
            .with_pigeon2_configs(self.pigeon_configs)
        )

        self.front_left_drive_motor_id = 21
        self.front_left_steer_motor_id = 22
        self.front_left_encoder_id = 23
        self.front_left_encoder_offset: units.rotation = -0.15380859375


        self.front_left_x_pos: units.meter = inchesToMeters(12.5)
        self.front_left_y_pos: units.meter = inchesToMeters(12.5)

        self.front_right_drive_motor_id = 31
        self.front_right_steer_motor_id = 32
        self.front_right_encoder_id = 33
        self.front_right_encoder_offset: units.rotation = 0.250244140625


        self.front_right_x_pos: units.meter = inchesToMeters(12.5)
        self.front_right_y_pos: units.meter = inchesToMeters(-12.5)

        self.back_left_drive_motor_id = 11
        self.back_left_steer_motor_id = 12
        self.back_left_encoder_id = 13
        self.back_left_encoder_offset: units.rotation = 0.314208984375


        self.back_left_x_pos: units.meter = inchesToMeters(-12.5)
        self.back_left_y_pos: units.meter = inchesToMeters(12.5)

        self.back_right_drive_motor_id = 41
        self.back_right_steer_motor_id = 42
        self.back_right_encoder_id = 43
        self.back_right_encoder_offset: units.rotation = 0.33642578125


        self.steer_motor_inverted = False
        self.encoder_inverted = True
        self.offset: units.meter = inchesToMeters(12.5)

        self.steer_profile = SmartProfile(
            "steer",
            {
                "kP": 3.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.14,
                "kV": 0.375,
                "kA": 0.0,
            },
            True,
        )
        self.drive_profile = SmartProfile(
            "drive",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.17,
                "kV": 0.104,
            },
            True,
        )

        self.controller = LemonInput(0)

    def teleopPeriodic(self):
        self.drivetrain.drive_robot_centric(
            self.controller.getLeftX(),
            self.controller.getLeftY(),
            self.controller.getRightX(),
        )
