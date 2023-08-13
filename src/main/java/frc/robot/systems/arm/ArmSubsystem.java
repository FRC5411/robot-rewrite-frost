package frc.robot.systems.arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Objects;
import frc.robot.systems.arm.ArmVars.Sets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import frc.robot.systems.arm.ArmVars.Sets.armPositions;

public class ArmSubsystem extends SubsystemBase {
    double manualTargetTheta;

    BooleanSupplier joint1Deadzone;
    BooleanSupplier joint2Deadzone;

    double stage1Setpoint;
    double stage2Setpoint;
    double stage3Setpoint;

    public ArmIO IO;

  public ArmSubsystem() {
    IO = new ArmIO();

    armPositions.setPositionMap();

    joint1Deadzone = () -> {return Objects.jointStageOne.inDeadzone(ArmVars.Constants.kJointAngleDeadzone);};
    joint2Deadzone = () -> {return Objects.jointStageTwo.inDeadzone(ArmVars.Constants.kJointAngleDeadzone);};

    ArmPosition idle = armPositions.positionMap.get(positions.Idle);

    stage1Setpoint = idle.getStage1OffsetAngle();
    stage2Setpoint = idle.getStage2OffsetAngle();
    stage3Setpoint = idle.getStage3OffsetAngle();
  }

  private void updateSetPoints (double stage1Angle, double stage2Angle, double stage3Angle) {
      stage1Setpoint = (stage1Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
      stage2Setpoint = (stage2Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
      stage3Setpoint = (stage3Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
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
      () -> {
        Objects.jointStageOne.resetProfiles();
        Objects.jointStageTwo.resetProfiles();
        Objects.jointStageThree.resetProfiles();
      },
      () -> {
        Objects.jointStageOne.executeControl(() -> stage1.getAsDouble());
        Objects.jointStageTwo.executeControl(() -> stage2.getAsDouble());
        Objects.jointStageThree.executeControl(() -> stage3.getAsDouble());
      },
      interrupted -> {},
      () -> false, 
      this); 
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

  @Override
  public void periodic() {
    IO.telemetry();
  }
}