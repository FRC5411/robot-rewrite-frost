package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars;

import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.systems.intake.IntakeVars.GamePieces;

import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.leds.LedSubsytem;

import frc.robot.managers.SuperStructureManager;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
  DriveSubsystem robotDrive;
  ArmSubsystem robotArm;
  IntakeSubsystem robotIntake;
  LedSubsytem LEDs;

  SuperStructureManager armIntakeManager;

  Visualizer visualizer;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();
    LEDs = new LedSubsytem();

    armIntakeManager = new SuperStructureManager(robotArm, robotIntake, LEDs);

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

    armIntakeManager.scheduleMode(GamePieces.Cone);
  }

  private void configureBindings() {


    

    ControllerVars.cubeModeBtn.onTrue(armIntakeManager.scheduleMode(GamePieces.Cube));
    ControllerVars.coneModeBtn.onTrue(armIntakeManager.scheduleMode(GamePieces.Cone));
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

  public SuperStructureManager getArmIntakeManager() {
    return armIntakeManager;
  }

  public Visualizer getVisualizer() {
    return visualizer;
  }

  public DriveSubsystem getDrive() {
    return robotDrive;
  }
}
