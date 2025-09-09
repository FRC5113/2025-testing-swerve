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

import math
from pathlib import Path

import wpilib
from wpilib import (
    Field2d,
    SmartDashboard,
    DataLogManager,
    CameraServer,
    Mechanism2d,
    MechanismLigament2d,
    Color8Bit,
    RobotController,
    DigitalInput,
    DutyCycleEncoder,
    DriverStation,
    RobotBase,
    PowerDistribution,
)

from wpimath import units, applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import (
    Transform3d,
    Pose2d,
    Transform2d,
    Rotation2d,
    Rotation3d,
    Translation2d,
)

from wpinet import WebServer

from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from rev import SparkMax, SparkLowLevel
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

import magicbot
from magicbot import feedback

from lemonlib import LemonInput, LemonCamera
from lemonlib.util import (
    curve,
    AlertManager,
    AlertType,
    LEDController,
    SnapX,
    SnapY,
    is_red,
)
from lemonlib.smart import SmartPreference, SmartProfile

from components.elevator import Elevator, ElevatorHeight
from components.claw import Claw, ClawAngle
from components.climber import Climber
from components.arm_control import ArmControl

from lemonlib import LemonRobot, fms_feedback
from lemonlib.command import CommandLemonInput
from lemonlib.util import get_file





    arm_control: ArmControl
    elevator: Elevator
    claw: Claw
    climber: Climber
    drivetrain: SwerveDrive

    # max_speed = SmartPreference(value=4.73)
    # max_angular_rate = SmartPreference(value=4.71)

    max_speed = 4.73
    max_angular_rate = 4.71

    def createObjects(self):
        """
        SWERVE
        """

        self.steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
        self.drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

        self.drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
        self.steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

        self.steer_feedback_type = swerve.SteerFeedbackType.REMOTE_CANCODER

        self.slip_current: units.amperes = 120.0

        self.drive_initial_configs = configs.TalonFXConfiguration()
        self.steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_stator_current_limit_enable(True)
        )
        self.encoder_initial_configs = configs.CANcoderConfiguration()
        self.pigeon_configs: configs.Pigeon2Configuration | None = None


        self.canbus = CANBus("rio", "./logs/example.hoot")

        self.speed_at_12_volts: units.meters_per_second = 4.73

        self.couple_ratio = 3.5714285714285716

        self.drive_gear_ratio = 6.746031746031747
        self.steer_gear_ratio = 21.428571428571427
        self.wheel_radius: units.meters = inchesToMeters(2)

        self.invert_left_side = False
        self.invert_right_side = True

        self.pigeon_id = 30

        self.steer_inertia: units.kilogram_square_meters = 0.01
        self.drive_inertia: units.kilogram_square_meters = 0.01
        self.steer_friction_voltage: units.volts = 0.2
        self.drive_friction_voltage: units.volts = 0.2

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
        self.encoder_offsets: list[float] = [
            -0.15380859375,
            0.250244140625,
            0.314208984375,
            0.33642578125,
        ]

        self.x_positions: list[units.meters] = [
            inchesToMeters(12.5),
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
            inchesToMeters(-12.5),
        ]
        self.y_positions: list[units.meters] = [
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
            inchesToMeters(12.5),
            inchesToMeters(-12.5),
        ]

        self.steer_motor_inverted = False
        self.encoder_inverted = True
        self.offset: units.meters = inchesToMeters(12.5)

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
        ELEVATOR
        """

        # hardware
        BRUSHLESS = SparkLowLevel.MotorType.kBrushless
        self.elevator_right_motor = SparkMax(59, BRUSHLESS)
        self.elevator_left_motor = SparkMax(57, BRUSHLESS)
        self.elevator_right_encoder = self.elevator_right_motor.getEncoder()
        self.elevator_left_encoder = self.elevator_left_motor.getEncoder()
        self.elevator_upper_switch = DigitalInput(0)
        self.elevator_lower_switch = DigitalInput(1)

        # physical constants
        self.elevator_carriage_mass = 15.0
        self.elevator_min_height = 0.0
        self.elevator_max_height = 0.7
        self.elevator_gearing = 10.0
        self.elevator_spool_radius: units.meters = 0.0223

        # profile (estimated)
        self.elevator_profile = SmartProfile(
            "elevator",
            {
                "kP": 50.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
                "kG": 0.0,
                "kMaxV": 2.0,
                "kMaxA": 20.0,
            },
            not self.low_bandwidth,
        )
        self.elevator_tolerance = 0.02

        """
        CLAW
        """

        # hardware
        self.claw_hinge_motor = SparkMax(56, BRUSHLESS)
        self.claw_left_motor = SparkMax(55, SparkLowLevel.MotorType.kBrushed)
        self.claw_right_motor = SparkMax(58, SparkLowLevel.MotorType.kBrushed)
        self.claw_hinge_encoder = self.claw_hinge_motor.getAbsoluteEncoder()
        self.claw_intake_limit = self.claw_left_motor.getReverseLimitSwitch()

        # physical constants
        self.claw_gearing = 82.5

        # profile (estimated)
        self.claw_profile = SmartProfile(
            "claw",
            {
                "kP": 0.15,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
                "kG": 0.0,
                "kMaxV": 150.0,
                "kMaxA": 500.0,
            },
            not self.low_bandwidth,
        )
        self.claw_hinge_tolerance = 3.0

        """
        CLIMBER
        """

        # hardware
        self.climber_motor = TalonFX(51)
        self.climber_encoder = DutyCycleEncoder(2)


        """
        MISCELLANEOUS
        """

        # self.period: units.seconds = 0.02

        self.leds = LEDController(0, 112)  # broken amount is 46

        self.pigeon = Pigeon2(30,canbus="can0")

        self.fms = DriverStation.isFMSAttached()

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )


        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.pdh = PowerDistribution()

        self.estimated_field = Field2d()
        # CameraServer().launch()

        self.arm_visuize = Mechanism2d(20, 50)
        self.arm_root = self.arm_visuize.getRoot("Arm Root", 10, 0)
        self.elevator_ligament = self.arm_root.appendLigament("Elevator", 5, 90)
        self.claw_ligament = self.elevator_ligament.appendLigament(
            "Claw", 5, 0, color=Color8Bit(0, 150, 0)
        )
        SmartDashboard.putData("Arm", self.arm_visuize)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True
        else:
            self.alliance = False
        # self.webserver = WebServer.getInstance()
        # if not DriverStation.isFMSAttached():
        #     self.webserver.start(
        #         port=5800,
        #         path=str(
        #             Path(__file__).parent.resolve() / "deploy/elastic-layout.json"
        #         ),
        #     )

    def disabledPeriodic(self):
        self.leds.move_across((5, 5, 0), 20, 20)

    def enabledperiodic(self):
        self.arm_control.engage()
    
    def teleopInit(self):
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)

        # self.commandprimary = CommandLemonInput(0

        self.upper_algae_button_released = True
        self.lower_algae_button_released = True

    def teleopPeriodic(self):
        self.drivetrain.drive_field_centric(
            applyDeadband(self.primary.getLeftX(),0.03),
            applyDeadband(self.primary.getLeftY(),0.03),
            applyDeadband(self.primary.getRightX(),0.03),

        )
        with self.consumeExceptions():
            """
            ARM
            """
            if self.elevator.error_detected():
                self.arm_control.next_state("elevator_failsafe")
            if self.secondary.getAButton():
                self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
            if self.secondary.getBButton():
                self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
                # DON'T UNCOMMENT!!!!!!!!!!!!!!!!!!!!!!!!
                # if self.camera_front.get_best_tag() is not None and (
                #     self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.565, -0.21, Rotation2d()) < 0.03
                #         )
                #     )
                #     or self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.55, 0.21, Rotation2d()) < 0.03
                #         )
                #     )
                # ):
                #     self.led_strip.is_aligned()
            if self.secondary.getXButton():
                # if self.camera_front.get_best_tag() is not None and (
                #     self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.565, -0.21, Rotation2d()) < 0.03
                #         )
                #     )
                #     or self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.55, 0.21, Rotation2d()) < 0.03
                #         )
                #     )
                # ):
                #     self.led_strip.is_aligned()
                self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
            if self.secondary.getYButton():

                # if self.camera_front.get_best_tag() is not None and (
                #     self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.53, -0.21, Rotation2d()) < 0.03
                #         )
                #     )
                #     or self.swerve_drive.get_distance_from_pose(
                #         self.camera_front.get_best_pose(True).transformBy(
                #             Transform2d(0.53, 0.21, Rotation2d()) < 0.03
                #         )
                #     )
                # ):
                #     self.led_strip.is_aligned()
                self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)

            if self.secondary.getStartButton():
                self.arm_control.set(
                    ElevatorHeight.STATION_CLOSE, ClawAngle.STATION_CLOSE
                )
            if self.secondary.getBackButton():
                self.arm_control.set(ElevatorHeight.STATION_FAR, ClawAngle.STATION_FAR)

            if self.secondary.getLeftTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    8.0 * applyDeadband(self.secondary.getLeftTriggerAxis(), 0.1)
                )
            if self.secondary.getRightTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    6.0 * applyDeadband(-self.secondary.getRightTriggerAxis(), 0.1)
                )

            if self.secondary.getRightBumper():
                self.arm_control.set_wheel_voltage(-1)

            # if self.secondary.getPOV() == 270:
            #     self.arm_control.next_state_now("elevator_failsafe")

            self.elevator_ligament.setLength((self.elevator.get_height() * 39.37) + 5)
            self.claw_ligament.setAngle(self.claw.get_angle() - 90)

        with self.consumeExceptions():
            """
            CLIMBER
            """

            if self.primary.getTriangleButton():
                self.climber.set_speed(-1)
            if self.primary.getCrossButton():
                self.climber.set_speed(1)
