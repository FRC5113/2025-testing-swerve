#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

from wpilib import RobotController
from phoenix6 import utils
from components.swerve_drivetrain import SwerveDrive

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.swerve = SwerveDrive()

    def update_sim(self, now: float, tm_diff: float) -> None:
        current_time = utils.get_current_time_seconds()
        last_sim_time = current_time
        delta_time = current_time - last_sim_time


        # use the measured time delta, get battery voltage from WPILib
        self.swerve.update_sim_state(
            delta_time, RobotController.getBatteryVoltage()
        )
