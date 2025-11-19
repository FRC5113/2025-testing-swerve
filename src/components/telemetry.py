from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units, hardware
from wpilib import (
    Color,
    Color8Bit,
    Mechanism2d,
    MechanismLigament2d,
    SmartDashboard,
    Field2d,
)
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState
from wpiutil import SendableBuilder, Sendable
from lemonlib.smart.nettables import SmartNT


class Telemetry(Sendable):
    max_speed: units.meters_per_second
    estimated_field: Field2d

    state = swerve.SwerveDrivetrain.SwerveDriveState()

    def __init__(self) -> None:
        Sendable.__init__(self)
        self._inst = NetworkTableInstance.getDefault()
        self._drive_state_table = self._inst.getTable("DriveState")
        self._drive_speeds = self._drive_state_table.getStructTopic(
            "Speeds", ChassisSpeeds
        ).publish()
        self._drive_module_states = self._drive_state_table.getStructArrayTopic(
            "ModuleStates", SwerveModuleState
        ).publish()
        self._drive_module_targets = self._drive_state_table.getStructArrayTopic(
            "ModuleTargets", SwerveModuleState
        ).publish()
        self._drive_module_positions = self._drive_state_table.getStructArrayTopic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self._drive_timestamp = self._drive_state_table.getDoubleTopic(
            "Timestamp"
        ).publish()
        self._drive_odometry_frequency = self._drive_state_table.getDoubleTopic(
            "OdometryFrequency"
        ).publish()

    def setup(self):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        SignalLogger.start()
        self.nt = SmartNT("Telemetry")

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        self.swerve_module_states = state.module_states

        self._drive_speeds.set(state.speeds)
        self._drive_module_states.set(state.module_states)
        self._drive_module_targets.set(state.module_targets)
        self._drive_module_positions.set(state.module_positions)
        self._drive_timestamp.set(state.timestamp)
        self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # Also write to log file
        pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]
        module_states_array = []
        module_targets_array = []
        for i in range(4):
            module_states_array.append(state.module_states[i].angle.degrees())
            module_states_array.append(state.module_states[i].speed)
            module_targets_array.append(state.module_targets[i].angle.degrees())
            module_targets_array.append(state.module_targets[i].speed)

        SignalLogger.write_double_array("DriveState/Pose", pose_array)
        SignalLogger.write_double_array("DriveState/ModuleStates", module_states_array)
        SignalLogger.write_double_array(
            "DriveState/ModuleTargets", module_targets_array
        )
        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )

        self.estimated_field.setRobotPose(state.pose)
        SmartDashboard.putData("Estimated Field", self.estimated_field)

        SmartDashboard.putData("Swerve Drive", self)

    def get_pose(self) -> Pose2d:
        return self.state.pose

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Robot Angle",
            # Rotate to match field widget
            lambda: self.state.pose.rotation().degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerve_module_states[0].speed * self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerve_module_states[0].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerve_module_states[1].speed * self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerve_module_states[1].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerve_module_states[2].speed * self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerve_module_states[2].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerve_module_states[3].speed * self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerve_module_states[3].angle.degrees(),
            lambda _: None,
        )

    def execute(self):
        pass
