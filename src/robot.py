from lemonlib import LemonRobot

# from components.telemetry import Telemetry

from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from lemonlib import LemonInput

from lemonlib.smart import SmartPreference, SmartProfile

from phoenix6 import CANBus, configs, hardware, signals, swerve, units
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration

from components.swerve_drivetrain import SwerveDrive

from wpimath.units import inchesToMeters
from wpilib import RobotBase
from lemonlib.smart import SmartProfile



class MyRobot(LemonRobot):
    drivetrain: SwerveDrive

    max_speed = SmartPreference(4.73)
    max_angular_rate = SmartPreference(4.71)

    def createObjects(self):
        """
        SWERVE
        """

        self.steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
        self.drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

        self.drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
        self.steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

        self.steer_feedback_type = swerve.SteerFeedbackType.REMOTE_CANCODER

        self.slip_current: units.ampere = 120.0

        self.drive_initial_configs = configs.TalonFXConfiguration()
        self.steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_stator_current_limit_enable(True)
        )
        self.encoder_initial_configs = configs.CANcoderConfiguration()
        self.pigeon_configs: configs.Pigeon2Configuration | None = None

        self.canbus = CANBus("can0", "./logs/example.hoot")

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
        # 0 = front left 1 = front right 2 = back left 3 = back right
        self.drive_motor_ids = [21, 31, 11, 41]
        self.steer_motor_ids = [22, 32, 12, 42]
        self.encoder_ids = [23, 33, 13, 43]
        self.encoder_offsets: list[units.rotation] = [
            -0.15380859375,
            0.250244140625,
            0.314208984375,
            0.33642578125,
        ]

        self.x_positions: list[units.meter] = [
            inchesToMeters(12.5),
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
            inchesToMeters(-12.5),
        ]
        self.y_positions: list[units.meter] = [
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
        ]

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
        self.module_constants = []
        for i in range(4):
            module_constant = self.constants_creator.create_module_constants(
                self.steer_motor_ids[i],
                self.drive_motor_ids[i],
                self.encoder_ids[i],
                self.encoder_offsets[i],
                self.x_positions[i],
                self.y_positions[i],
                self.invert_left_side if i in [0, 2] else self.invert_right_side,
                self.steer_motor_inverted,
                self.encoder_inverted,
            )
            self.module_constants.append(module_constant)

        self.front_left = self.module_constants[0]
        self.front_right = self.module_constants[1]
        self.back_left = self.module_constants[2]
        self.back_right = self.module_constants[3]

        """
        MISC
        """

        self.controller = LemonInput(0)

    def teleopPeriodic(self):
        self.drivetrain.drive_field_centric(
            self.controller.getRawAxis(0),
            self.controller.getRawAxis(1),
            self.controller.getRawAxis(2),
        )
