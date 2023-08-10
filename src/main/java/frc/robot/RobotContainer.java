package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;
import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.systems.intake.IntakeVars.GamePieces;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.managers.ArmIntakeManager;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.leds.LedSubsytem;

public class RobotContainer {
  DriveSubsystem robotDrive;
  ArmSubsystem robotArm;
  IntakeSubsystem robotIntake;
  LedSubsytem LEDs;

  ArmIntakeManager armIntakeManager;

  Visualizer visualizer;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();
    LEDs = new LedSubsytem();

    armIntakeManager = new ArmIntakeManager(robotArm, robotIntake, LEDs);

    visualizer = new Visualizer();

    configureBindings();

    robotDrive.setDefaultCommand(
        robotDrive.driveCMD(
            () -> - deadzone(ControllerVars.xboxController.getLeftY()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getLeftX()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> deadzone(ControllerVars.xboxController.getRightX()) * DriveVars.Constants.kMaxRotMeters,
            () -> RobotStates.sField
    ));

    robotArm.setDefaultCommand(robotArm.moveToPositionCmd(() -> false));

    robotIntake.setDefaultCommand(robotIntake.DEFspinSlowCommand());
  }

  private void configureBindings() {
    // ControllerVars.resetOdometryBtn.onTrue(robotDrive.resetPoseCMD(new Pose2d())); // Reset odometry to current position
    // ControllerVars.toggleRobotOrientBtn.onTrue(robotDrive.toggleFieldCMD());
    // // engageLimeLightBtn.onTrue(new InstantCommand(() -> m_swerve.PPmoveToPositionCommand().schedule()));
    // ControllerVars.engageAutobalanceBtn.whileTrue(robotDrive.autoBalanceCMD());

    // ControllerVars.engageAutobalanceBtn.onTrue(armIntakeManager.setMode(GamePieces.Cone));
    // ControllerVars.engageAutobalanceBtn.onFalse(armIntakeManager.setMode(GamePieces.Cube));

    // ControllerVars.resetOdometryBtn.onTrue(new InstantCommand(() -> armIntakeManager.manualIntakeCommand().schedule()));

    // ControllerVars.engageLimeLightBtn.onTrue(new InstantCommand(() -> armIntakeManager.outTakeCommand().schedule()));

    ControllerVars.substationPickupBtn.whileTrue(armIntakeManager.goToSubstation());
    ControllerVars.substationPickupBtn.onFalse(
      armIntakeManager.manualIntakeCommand().alongWith(armIntakeManager.goToIdle()));
    
    ControllerVars.floorPickupBtn.whileTrue(armIntakeManager.goToPickup());
    ControllerVars.floorPickupBtn.onFalse(armIntakeManager.manualIntakeCommand());

    ControllerVars.scoreHighBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToHighScore()));
    ControllerVars.scoreHighBtn.onFalse(armIntakeManager.goToIdle());
    ControllerVars.scoreHighBtn.onFalse(armIntakeManager.manualIntakeCommand());

    ControllerVars.altFloorPickupBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToPickUpAlt()));
    ControllerVars.altFloorPickupBtn.onFalse(
      armIntakeManager.manualIntakeCommand().alongWith(armIntakeManager.goToIdle()));

    ControllerVars.scoreMidBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToMidScore()));
    ControllerVars.scoreMidBtn.onFalse(armIntakeManager.manualIntakeCommand());
    ControllerVars.scoreMidBtn.onFalse(armIntakeManager.goToIdle());

    ControllerVars.scoreLowBtn.onTrue(armIntakeManager.goToLowScore());
    ControllerVars.scoreLowBtn.onFalse(armIntakeManager.manualIntakeCommand());
    ControllerVars.scoreLowBtn.onFalse(armIntakeManager.goToIdle());

    ControllerVars.placeIdleBtn.onTrue(new SequentialCommandGroup(
      armIntakeManager.outTakeCommand(),
      new WaitCommand(0.25),
      armIntakeManager.goToIdle()
    ));
    ControllerVars.placeIdleBtn.onFalse(getAutonomousCommand());
    ControllerVars.placeIdleBtn.onFalse(robotIntake.spinOffCommand());

    ControllerVars.coneModeBtn.onTrue(armIntakeManager.setMode(GamePieces.Cone));
    ControllerVars.cubeModeBtn.onTrue(armIntakeManager.setMode(GamePieces.Cube));
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

  public ArmIntakeManager getArmIntakeManager() {
    return armIntakeManager;
  }

  public Visualizer getVisualizer() {
    return visualizer;
  }
}
