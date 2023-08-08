package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.arm.ArmVars.Sets.armPositions;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;

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

    configureBindings();
  }

  private void configureBindings() {
    ControllerVars.resetOdometryBtn.onTrue(robotDrive.resetPoseCMD(new Pose2d())); // Reset odometry to current position
    ControllerVars.toggleRobotOrientBtn.onTrue(robotDrive.toggleFieldCMD());
    // engageLimeLightBtn.onTrue(new InstantCommand(() -> m_swerve.PPmoveToPositionCommand().schedule()));
    ControllerVars.engageAutobalanceBtn.whileTrue(robotDrive.autoBalanceCMD());


    
    copilotController.button(0)
        .whileTrue(m_arm.moveToPositionCommand(positions.Substation))
        .onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    copilotController.button(1)
        .whileTrue(m_arm.moveToPositionCommand(positions.Floor))
        .onFalse(m_claw.intakeCommand());

    copilotController.button(2)
        .onTrue(new InstantCommand( () -> m_arm.goToScoreHigh().schedule()))
        .onFalse(m_arm.defaultCommand())
        .onFalse(m_claw.intakeCommand());

    copilotController.button(3)
        .whileTrue(new InstantCommand( () -> m_arm.goToFloor().schedule()))
        .onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    copilotController.button(4)
        .whileTrue(new InstantCommand( () -> m_arm.goToScoreMid().schedule()))
        .onFalse(m_claw.intakeCommand())
        .onFalse(m_arm.defaultCommand());
    
    copilotController.button(5)
      .whileTrue(m_arm.moveToPositionCommand(positions.ScoreLow))
      .onFalse(m_claw.intakeCommand());
    
    copilotController.button(6)
      .onTrue(new SequentialCommandGroup((m_claw.outTakeCommand()), new WaitCommand(0.25), m_arm.moveToPositionCommand(positions.Idle)))
      .onFalse(m_claw.spinOffCommand())
      .onFalse(m_claw.spinOffCommand());
    
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

  public static positions getPosition() {
      return robotArm.getPos();
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
