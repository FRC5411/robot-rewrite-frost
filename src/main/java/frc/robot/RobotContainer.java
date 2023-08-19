package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;

import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.systems.intake.IntakeVars.GamePieces;

import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import frc.robot.systems.leds.LedSubsytem;

import frc.robot.managers.ArmIntakeManager;

import edu.wpi.first.wpilibj.DriverStation;

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
            () -> - deadzone(ControllerVars.xboxController.getLeftX()) * DriveVars.Constants.kMaxLinSpeedMeters,
            () -> - deadzone(ControllerVars.xboxController.getRightX()) * DriveVars.Constants.kMaxRotMeters,
            () -> RobotStates.sField
    ));

    robotArm.setDefaultCommand(
      robotArm.moveToPositionCmd(
        () -> robotArm.getStage1Setpoint(),
        () -> robotArm.getStage2Setpoint(),
        () -> robotArm.getStage3Setpoint()));

    // robotIntake.setDefaultCommand(robotIntake.DEFspinSlowCommand());

    armIntakeManager.setMode(GamePieces.Cone).schedule();
  }

  private void configureBindings() {
    // ControllerVars.resetOdometryBtn.onTrue(robotDrive.resetPoseCMD(new Pose2d())); // Reset odometry to current position
    // ControllerVars.toggleRobotOrientBtn.onTrue(robotDrive.toggleFieldCMD());
    // engageLimeLightBtn.onTrue(new InstantCommand(() -> m_swerve.PPmoveToPositionCommand().schedule()));
    // ControllerVars.engageAutobalanceBtn.whileTrue(robotDrive.autoBalanceCMD());

    ControllerVars.resetOdometryBtn.onTrue(new InstantCommand(() -> robotDrive.getIO().resetModules()));

    // TODO: ADD MANUAL
    // REAL ARM TESTING BINDS
    // A
    // ControllerVars.resetOdometryBtn.onTrue(robotArm.updateSetPointsCMD(positions.ScoreMidCone));
    // // B
    // ControllerVars.engageAutobalanceBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToMidScore().schedule()));
    // // X
    // ControllerVars.engageLimeLightBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToHighScore().schedule()));
    // // Y
    // ControllerVars.engageAutobalanceBtn.onTrue(new InstantCommand(() -> new InstantCommand(armIntakeManager.scheduleDefaultCMD()).schedule()));   
    
    // INTAKE TESTINGS
    // ControllerVars.resetOdometryBtn.onTrue(armIntakeManager.setMode(GamePieces.Cone));
    // ControllerVars.engageAutobalanceBtn.onFalse(armIntakeManager.setMode(GamePieces.Cube));

    // ControllerVars.resetOdometryBtn.onTrue(new InstantCommand(() -> armIntakeManager.manualIntakeCommand().schedule()));

    // ControllerVars.engageLimeLightBtn.onTrue(new InstantCommand(() -> armIntakeManager.outTakeCommand().schedule()));

    // ARM TESTINGS
    // ControllerVars.resetOdometryBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToHighScore()));
    // ControllerVars.resetOdometryBtn.onFalse(new InstantCommand(armIntakeManager.scheduleDefaultCMD()));
    // ControllerVars.resetOdometryBtn.onFalse(armIntakeManager.manualIntakeCommand());

    // ControllerVars.toggleRobotOrientBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToPickUpAlt()));
    // ControllerVars.toggleRobotOrientBtn.onFalse(
    //   armIntakeManager.manualIntakeCommand().alongWith(new InstantCommand(armIntakeManager.scheduleDefaultCMD())));

    // ControllerVars.engageAutobalanceBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToMidScore()));
    // ControllerVars.engageAutobalanceBtn.onFalse(armIntakeManager.manualIntakeCommand());
    // ControllerVars.engageAutobalanceBtn.onFalse(new InstantCommand(armIntakeManager.scheduleDefaultCMD()));

    // ControllerVars.engageLimeLightBtn.onTrue(armIntakeManager.goToLowScore());
    // ControllerVars.engageLimeLightBtn.onFalse(armIntakeManager.manualIntakeCommand());
    // ControllerVars.engageLimeLightBtn.onFalse(new InstantCommand(armIntakeManager.scheduleDefaultCMD()));

    ControllerVars.substationPickupBtn.whileTrue(
      armIntakeManager.goToSubstation().andThen(
        new InstantCommand(() -> {
          if(armIntakeManager.getGP() == GamePieces.Cone) armIntakeManager.detectIntakeCommand().schedule();
          else armIntakeManager.manualIntakeCommand().schedule();})));
    ControllerVars.substationPickupBtn.onFalse(
      armIntakeManager.manualIntakeCommand().alongWith(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(false))));
    
    ControllerVars.floorPickupBtn.whileTrue(armIntakeManager.goToPickup().andThen(
      new InstantCommand(() -> {
        if(armIntakeManager.getGP() == GamePieces.Cone) armIntakeManager.detectIntakeCommand().schedule();
        else armIntakeManager.manualIntakeCommand().schedule();})));
    ControllerVars.floorPickupBtn.onFalse(
      armIntakeManager.manualIntakeCommand().alongWith(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(false))));

    ControllerVars.scoreHighBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToHighScore().schedule()));
    ControllerVars.scoreHighBtn.onFalse(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(true)));
    // ControllerVars.scoreHighBtn.onFalse(armIntakeManager.manualIntakeCommand());

    ControllerVars.altFloorPickupBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToPickUpAlt().schedule()).andThen(
      new InstantCommand(() -> {
        if(armIntakeManager.getGP() == GamePieces.Cone) armIntakeManager.detectIntakeCommand().schedule();
        else armIntakeManager.manualIntakeCommand().schedule();})));
    ControllerVars.altFloorPickupBtn.onFalse(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(false)));

    ControllerVars.scoreMidBtn.onTrue(new InstantCommand(() -> armIntakeManager.goToMidScore().schedule()));
    ControllerVars.scoreMidBtn.onFalse(armIntakeManager.manualIntakeCommand());
    ControllerVars.scoreMidBtn.onFalse(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(true)));

    ControllerVars.scoreLowBtn.onTrue(armIntakeManager.goToLowScore());
    ControllerVars.scoreLowBtn.onFalse(armIntakeManager.manualIntakeCommand());
    ControllerVars.scoreLowBtn.onFalse(new InstantCommand(() -> armIntakeManager.scheduleDefaultCMD(false)));

    ControllerVars.placeIdleBtn
      .onTrue(armIntakeManager.outTakeCommand());
      // .onFalse(robotIntake.spinOffCommand());

    ControllerVars.coneModeBtn.onTrue(new InstantCommand(() -> armIntakeManager.setMode(GamePieces.Cube).schedule()));
    ControllerVars.cubeModeBtn.onTrue(new InstantCommand(() -> armIntakeManager.setMode(GamePieces.Cube).schedule()));

    // ControllerVars.copilotController.button(12).onTrue(new InstantCommand( () -> {
    //   if (ControllerVars.copilotController.isButtonDown(9)) {
    //     robotIntake.toggleCMD(); 
    //   }
    // }));
    // ControllerVars.copilotController.button(14).whileTrue(new InstantCommand( () -> {
    //   if (ControllerVars.copilotController.isButtonDown(9)) {
    //     robotIntake.spinOffCommand();
    //   }
    // }));
    // ControllerVars.copilotController.button(14).onFalse(new InstantCommand( () -> {
    //   if (ControllerVars.copilotController.isButtonDown(9)) 
    //   robotIntake.spinOff();
    // }));
    // ControllerVars.copilotController.button(13).whileTrue(new InstantCommand( () -> {
    //   if (ControllerVars.copilotController.isButtonDown(9)) {
    //     robotIntake.spinIn();
    //   }
    // }));
    // ControllerVars.copilotController.button(13).onFalse(new InstantCommand( () -> {if (ControllerVars.copilotController.isButtonDown(9)) m_claw.spinOff();}));
  }

  public Command  getAutonomousCommand() {
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

  public DriveSubsystem getDrive() {
    return robotDrive;
  }
}
