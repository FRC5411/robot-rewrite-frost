package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
  DriveSubsystem robotDrive;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();

    robotDrive.setDefaultCommand(
        robotDrive.driveCMD(
            () -> - deadzone(ControllerVars.xboxController.getLeftY()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getLeftX()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getRightX()) * DriveVars.Constants.kMaxRotMeters,
            () -> RobotStates.sField
    ));

    configureBindings();
  }

  private void configureBindings() {
    ControllerVars.b.onTrue(robotDrive.instantdriveCMD(() -> 5.4, () -> 0, () -> 0, () -> false));
    ControllerVars.b.onFalse(robotDrive.instantdriveCMD(() -> 0, () -> 0, () -> 0, () -> false));
    ControllerVars.a.onTrue(robotDrive.instantdriveCMD(() -> 0, () -> 1, () -> 0, () -> false));
    ControllerVars.a.onFalse(robotDrive.instantdriveCMD(() -> 0, () -> 0, () -> 0, () -> false));
  }

  public Command getAutonomousCommand() {
    return robotDrive.getAuton();
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

  public double deadzone(double val) {
    if (Math.abs(val) < 0.1) {
      return 0;
    } else {
      return val;
    }
  }

  public static String getAutonPath() {
    return RobotStates.sAutonPath;
  }
}
