#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from commands2 import Command, RunCommand, SwerveControllerCommand
from commands2.button import CommandXboxController
from subsystems.drive import Drive
from subsystems.roller import Roller
from commands.game import Game
from commands.auto import Auto


import wpimath
import enum
import constants


ControllersReference = constants.Controllers
DriveSubsystemReference = constants.Subsystems.Drive
RollerSubsystemReference = constants.Subsystems.Roller



class RobotContainer:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # The enum used as keys for selecting the command to run.
    class CommandSelector(enum.Enum):
        ONE = enum.auto()
        TWO = enum.auto()
        THREE = enum.auto()

    # An example selector method for the selectcommand.
    def select(self) -> CommandSelector:
        """Returns the selector that will select which command to run.
        Can base this choice on logical conditions evaluated at runtime.
        """
        return self.CommandSelector.ONE

    def __init__(self) -> None:
        self._initSubsystems()
        self._initControllers()
        self._initCommands()
        self._initControllerBindings()

    def _initSubsystems(self):
        """Initializes subsystems. Should only be called from __init__"""
        self._robotDrive = Drive()
        self._robotRoller = Roller()

    def _initControllers(self):
        self._driverController = CommandXboxController(
            ControllersReference.DriverPort
        )
        self._operatorController = CommandXboxController(
            ControllersReference.OperatorPort
        )

    def _initCommands(self):
        """Initializes commands. Should only be called from __init__"""
        self.game = Game(self)
        self.auto = Auto(self)

        # Set default commands for all subsystems
        self._robotDrive.setDefaultCommand(

            RunCommand(
                lambda: self._robotDrive.drive(
                    -wpimath.applyDeadband(self._driverController.getLeftY(), ControllersReference.kDriveDeadband),
                    -wpimath.applyDeadband(self._driverController.getLeftX(), ControllersReference.kDriveDeadband),
                    -wpimath.applyDeadband(self._driverController.getRightX(), ControllersReference.kDriveDeadband),
                    True
                ),
                self._robotDrive
            )
        )
        self._robotRoller.setDefaultCommand(
            self._robotRoller.stopCommand()
        )

    def _initControllerBindings(self) -> None:
        """Use this method to define your button->command mappings."""
        # All possible XBOX controller inputs are included below for easy reference.
        # Leaving unused, commented lines helps us keep track of what inputs aren't being used.
        # self.driver.rightStick().whileTrue(cmd.none())
        # self.driver.leftStick().whileTrue(cmd.none())
        # self.driver.leftTrigger().whileTrue(cmd.none())
        # self.driver.rightTrigger().whileTrue(cmd.none())
        # self.driver.rightBumper().whileTrue(cmd.none())
        # self.driver.leftBumper().whileTrue(cmd.none())
        # self.driver.povUp().whileTrue(cmd.none())
        # self.driver.povDown().whileTrue(cmd.none())
        # self.driver.povLeft().whileTrue(cmd.none())
        # self.driver.povRight().whileTrue(cmd.none())
        # self.driver.a().whileTrue(cmd.none())
        # self.driver.b().whileTrue(cmd.none())
        # self.driver.y().whileTrue(cmd.none())
        # self.driver.x().whileTrue(cmd.none())
        # self.driver.start().whileTrue(cmd.none())
        # self.driver.back().whileTrue(cmd.none())
        self._operatorController.rightTrigger().whileTrue(self._robotRoller.ejectCommand())
        # TODO: Bind the operator's left trigger to reverse

    def getAutonomousCommand(self) -> Command:
        """Use this to pass the autonomous command to the main {Robot} class.

        :returns: the command to run in autonomous
        """
        return self.auto.get()
