package frc.robot.systems.arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Constants;
import frc.robot.systems.arm.ArmVars.Objects;
import frc.robot.systems.arm.ArmVars.Sets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import frc.robot.systems.arm.ArmVars.Sets.armPositions;

public class ArmSubsystem extends SubsystemBase {
    double stage1Setpoint;
    double stage2Setpoint;
    double stage3Setpoint;

    public ArmIO IO;

  public ArmSubsystem() {
    IO = new ArmIO();

    armPositions.setPositionMap();

    ArmPosition idle = armPositions.positionMap.get(positions.Idle);

    stage1Setpoint = 0.0;
    stage2Setpoint = 5.0;
    stage3Setpoint = 0.0;
  }

  private void updateSetPoints (double stage1Angle, double stage2Angle, double stage3Angle) {
      stage1Setpoint = (stage1Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
      stage2Setpoint = (stage2Angle - Sets.stageTwoJoint.kArmOffsetDeg) % 360;
      stage3Setpoint = (stage3Angle - Sets.stageThreeJoint.kArmOffsetDeg) % 360;
  }

  public Command updateSetPointsCMD(armPositions.positions position) {
      ArmPosition pos = armPositions.positionMap.get(position);
      return updateSetPointsCMD(pos.getStage1Angle(), pos.getStage2Angle(), pos.getStage3Angle());
  }

  public Command updateSetPointsCMD(double x, double y, double z) {
      return new InstantCommand(() -> updateSetPoints(x, y, z));
  }

  // The bool suppliers are not used currently, but they are there for future use
  public Command moveToPositionCmd(DoubleSupplier stage1, DoubleSupplier stage2, DoubleSupplier stage3) {
    return new FunctionalCommand(
      () -> { resetAllProfiles(); },
      () -> {
        Objects.jointStageOne.executeControl(() -> stage1.getAsDouble());
        Objects.jointStageTwo.executeControl(() -> stage2.getAsDouble());
        Objects.jointStageThree.executeControl(() -> stage3.getAsDouble());
      },
      interrupted -> {},
      () -> false, 
      this); 
  }

  public void manualUpdate(double x, double y, double theta) {
      stage1Setpoint += x * Constants.kXSpeed;
      stage2Setpoint += y * Constants.kYSpeed;
      stage3Setpoint += theta * Constants.kThetaSpeed;

      stage1Setpoint %= 360;
      stage2Setpoint %= 360;
      stage3Setpoint %= 360;
  }

  public double getStage1Setpoint() {
    return stage1Setpoint;
  }

  public double getStage2Setpoint() {
    return stage2Setpoint;
  }

  public double getStage3Setpoint() {
    return stage3Setpoint;
  }

  public void resetAllProfiles() {
    Objects.jointStageOne.resetProfiles();
    Objects.jointStageTwo.resetProfiles();
    Objects.jointStageThree.resetProfiles();
  }

  @Override
  public void periodic() {
    IO.telemetry();
    SmartDashboard.putNumber("Arms/Stage 1 Setpoint", stage1Setpoint);
    SmartDashboard.putNumber("Arms/Stage 2 Setpoint", stage2Setpoint);
    SmartDashboard.putNumber("Arms/Stage 3 Setpoint", stage3Setpoint);
  }
}