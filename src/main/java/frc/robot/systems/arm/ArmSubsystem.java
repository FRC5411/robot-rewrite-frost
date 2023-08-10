package frc.robot.systems.arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Objects;
import frc.robot.systems.arm.ArmVars.Sets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
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

    public positions mPos;

  public ArmSubsystem() {
    armPositions.setPositionMap();

    joint1Deadzone = () -> {return Objects.jointStageOne.inDeadzone(ArmVars.Constants.kJointAngleDeadzone);};
    joint2Deadzone = () -> {return Objects.jointStageTwo.inDeadzone(ArmVars.Constants.kJointAngleDeadzone);};

    stage1Setpoint = Objects.jointStageOne.getOffsetEncValue();
    stage2Setpoint = Objects.jointStageTwo.getOffsetEncValue();
    stage3Setpoint = Objects.jointStageThree.getOffsetEncValue();

    mPos = positions.Idle;
  }

  private void updateSetPoints (double stage1Angle, double stage2Angle, double stage3Angle) {    
      manualTargetTheta = stage3Angle - Objects.jointStageThree.jointOffsetDeg;
      Objects.jointStageOne.jointSetpoint = (stage1Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
      Objects.jointStageTwo.jointSetpoint = (stage2Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
      Objects.jointStageThree.jointSetpoint = (stage3Angle - Sets.stageOneJoint.kArmOffsetDeg) % 360;
  }

  public Command updateSetPointsCMD(armPositions.positions position) {
      ArmPosition pos = armPositions.positionMap.get(position);
      mPos = position;
      return updateSetPointsCMD(pos.getStage1Angle(), pos.getStage2Angle(), pos.getStage3Angle());
  }

  public Command updateSetPointsCMD(double x, double y, double z) {
      return new InstantCommand(() -> updateSetPoints(x, y, z));
  }

  // The bool suppliers are not used currently, but they are there for future use
  public Command moveToPositionCmd(BooleanSupplier goingToTuck) {
    return new FunctionalCommand(
      () -> {
        Objects.jointStageOne.resetProfiles();
        Objects.jointStageTwo.resetProfiles();
        Objects.jointStageThree.resetProfiles();
      },
      () -> {
        Objects.jointStageOne.executeControl(() -> true, goingToTuck);
        Objects.jointStageTwo.executeControl(joint1Deadzone, goingToTuck);
        Objects.jointStageThree.executeControl(joint2Deadzone, goingToTuck);
      },
      interrupted -> {},
      () -> false, 
      this); 
  }

  @Override
  public void periodic() {}
}