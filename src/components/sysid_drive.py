from commands2.sysid import SysIdRoutine

from lemonlib.util import MagicSysIdRoutine
from components.swerve_drivetrain import SwerveDrive
from wpimath import units
from phoenix6 import SignalLogger, swerve
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve
from wpilib.sysid import SysIdRoutineLog


class SysIdDriveTranslation(MagicSysIdRoutine):
    drivetrain: SwerveDrive

    def setup(self):
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.setup_sysid(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                lambda log: None,
                self,
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drivetrain.apply_request(
            self.translation_characterization.with_volts(voltage)
        )


class SysIdDriveSteer(MagicSysIdRoutine):
    drivetrain: SwerveDrive

    def setup(self):
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.setup_sysid(
            SysIdRoutine.Config(
                stepVoltage=7.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                lambda log: None,
                self,
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drivetrain.apply_request(self.steer_characterization.with_volts(voltage))
        print(voltage)


class SysIdDriveRotation(MagicSysIdRoutine):
    drivetrain: SwerveDrive

    def setup(self):
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()
        self.setup_sysid(
            SysIdRoutine.Config(
                rampRate=math.pi / 6,
                stepVoltage=7.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                lambda log: None,
                self,
            ),
        )

    def drive_sysid(self, output) -> None:
        self.drivetrain.apply_request(
            self.rotation_characterization.with_rotational_rate(output)
        )
        SignalLogger.write_double("Rotational_Rate", output),
