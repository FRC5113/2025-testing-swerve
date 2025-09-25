import math

from lemonlib import (
    LemonRobot,
    LemonInput,
    LemonCamera,
    fms_feedback
)
from lemonlib.smart import (
    SmartPreference,
    SmartProfile,
)
from lemonlib.util import (
    AlertManager,
    AlertType,
)

from phoenix6 import (
    CANBus,
    configs,
    swerve,
    units,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
)
from wpimath.units import inchesToMeters
from wpimath import (
    units,
    applyDeadband,
)
from wpimath.geometry import (
    Transform3d,
    Rotation3d,
)
from wpilib import (
    Field2d,
    SmartDashboard,
)
from robotpy_apriltag import (
    AprilTagField,
    AprilTagFieldLayout,
)
from components.swerve_drivetrain import SwerveDrive
from components.telemetry import Telemetry
from components.sysid_drive import (
    SysIdDriveTranslation,
    SysIdDriveSteer,
    SysIdDriveRotation,
)
from components.odometry import Odometry
from autonomous.auto_base import AutoBase


class MyRobot(LemonRobot):
    sys_translation: SysIdDriveTranslation
    sys_steer: SysIdDriveSteer
    sys_rotation: SysIdDriveRotation

    odometry: Odometry
    telemetry: Telemetry
    drivetrain: SwerveDrive

    max_speed = SmartPreference(value=4.73)
    max_angular_rate = SmartPreference(value=4.71)

    def create_swerve(self):
        self.drive_controller = self.drive_profile.create_ctre_flywheel_controller()
        self.steer_controller = self.steer_profile.create_ctre_turret_controller()

        self.constants_creator.with_steer_motor_gains(self.steer_controller)
        self.constants_creator.with_drive_motor_gains(self.drive_controller)

        
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

        self.canbus = CANBus("can0", "./logs/example.hoot")

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
            0.350341796875,
            -0.245361328125,
            0.2060546875,
            -0.335205078125,
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

        self.steer_motor_inverted = True
        self.encoder_inverted = False


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
                "kA": 0.01
            },
            True,
        )
        self.translation_profile = SmartProfile(
            "translation",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            not self.low_bandwidth,
        )
        self.rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kMaxV": 10.0,
                "kMaxA": 100.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            not self.low_bandwidth,
        )
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
        self.create_swerve()

        """
        ODOMETRY
        """
        self.robot_to_camera_front = Transform3d(
            -0.2286,
            0.0,
            0.2667,
            Rotation3d(0.0, 0.0, math.pi),
        )
        self.robot_to_camera_back = Transform3d(
            -0.0381,
            0.0,
            0.762,
            Rotation3d(0.0, math.pi / 6, 0.0),
        )

        # self.field_layout = AprilTagFieldLayout(
        #     str(Path(__file__).parent.resolve() / "2025_test_field.json")
        # )
        self.field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2025ReefscapeWelded
        )

        self.camera_front = LemonCamera(
            "Global_Shutter_Camera", self.robot_to_camera_front, self.field_layout
        )
        self.camera_back = LemonCamera(
            "USB_Camera", self.robot_to_camera_back, self.field_layout
        )

        """
        MISCELLANEOUS
        """

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.estimated_field = Field2d()

    def on_enable(self):
        self.create_swerve()
        self.drivetrain.update_configs()

    def disabledPeriodic(self):
        self.odometry.execute()

    def teleopInit(self):
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0)
        SmartDashboard.putData("Primary Controller", self.primary)
        # self.secondary = LemonInput(1)
        # self.tertiary = LemonInput(2)

    def teleopPeriodic(self):
        with self.consumeExceptions():
            self.drivetrain.drive_field_centric(
                applyDeadband(self.primary.getLeftX(), 0.03),
                applyDeadband(self.primary.getLeftY(), 0.03),
                applyDeadband(self.primary.getRightX(), 0.03),
            )

        # with self.consumeExceptions():
        #     """
        #     SYSID
        #     """
        #     if self.tertiary.getLeftBumper():
        #         if self.tertiary.getAButton():
        #             self.sys_translation.quasistatic_forward()
        #         if self.tertiary.getBButton():
        #             self.sys_translation.quasistatic_reverse()
        #         if self.tertiary.getXButton():
        #             self.sys_translation.dynamic_forward()
        #         if self.tertiary.getYButton():
        #             self.sys_translation.dynamic_reverse()
        #     elif self.tertiary.getRightBumper():
        #         if self.tertiary.getAButton():
        #             self.sys_steer.quasistatic_forward()
        #         if self.tertiary.getBButton():
        #             self.sys_steer.quasistatic_reverse()
        #         if self.tertiary.getXButton():
        #             self.sys_steer.dynamic_forward()
        #         if self.tertiary.getYButton():
        #             self.sys_steer.dynamic_reverse()
        #     elif self.tertiary.getStartButton():
        #         if self.tertiary.getAButton():
        #             self.sys_rotation.quasistatic_forward()
        #         if self.tertiary.getBButton():
        #             self.sys_rotation.quasistatic_reverse()
        #         if self.tertiary.getXButton():
        #             self.sys_rotation.dynamic_forward()
        #         if self.tertiary.getYButton():
        #             self.sys_rotation.dynamic_reverse()

    def _display_auto_trajectory(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            selected_auto.display_trajectory()

    @fms_feedback
    def display_auto_state(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"
