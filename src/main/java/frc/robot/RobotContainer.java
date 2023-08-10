package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.systems.arm.ArmSubsystem;

public class RobotContainer {
  DriveSubsystem robotDrive;
  ArmSubsystem robotArm;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();


    robotDrive.setDefaultCommand(
        robotDrive.driveCMD(
            () -> - deadzone(ControllerVars.xboxController.getLeftY()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getLeftX()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getRightX()) * DriveVars.Constants.kMaxRotMeters,
            () -> RobotStates.sField
    ));

    robotArm.setDefaultCommand(robotArm.moveToPositionCmd(() -> false));

    configureBindings();
  }

  private void configureBindings() {
    ControllerVars.resetOdometryBtn.onTrue(robotDrive.resetPoseCMD(new Pose2d())); // Reset odometry to current position
    ControllerVars.toggleRobotOrientBtn.onTrue(robotDrive.toggleFieldCMD());
    // engageLimeLightBtn.onTrue(new InstantCommand(() -> m_swerve.PPmoveToPositionCommand().schedule()));
    ControllerVars.engageAutobalanceBtn.whileTrue(robotDrive.autoBalanceCMD());
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
