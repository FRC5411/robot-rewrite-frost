// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
  DriveSubsystem robotDrive;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();

    robotDrive.setDefaultCommand(
        robotDrive.driveCMD(
            () -> ControllerVars.xboxController.getLeftY(),
            () -> ControllerVars.xboxController.getLeftX(),
            () -> ControllerVars.xboxController.getRightX(),
            () -> RobotStates.sField
    ));

    configureBindings();
  }

  private void configureBindings() {
    ControllerVars.b.onTrue(robotDrive.resetPoseCMD(new Pose2d()));
    ControllerVars.a.onTrue(robotDrive.toggleFieldCMD());
    ControllerVars.x.onTrue(robotDrive.xLockCMD());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

    public static DriverStation.Alliance getDriverAlliance() {
      // What to do for competition
      //return DriverStation.getAlliance();

      // What to do for testing
      return DriverStation.Alliance.Red;
  }

  public static RobotStates.positions getPosition() {
      return RobotStates.positions.Floor;
  }

}
