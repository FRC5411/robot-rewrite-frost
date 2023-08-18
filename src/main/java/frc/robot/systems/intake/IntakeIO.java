package frc.robot.systems.intake;
import frc.robot.systems.intake.IntakeVars.Constants;
import frc.robot.systems.intake.IntakeVars.GamePieces;
import frc.robot.systems.intake.IntakeVars.Objects;
import frc.robot.utils.REVConfigs;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeIO {

  public IntakeIO() {
    REVConfigs.NEO550(IntakeVars.Objects.spinnerLeft, false);
    REVConfigs.NEO550(IntakeVars.Objects.spinnerRight, true);
//    follow();
  }

  public void closeGrip() {
    Objects.claw.set(Value.kForward);
  }

  public void openGrip() {
    Objects.claw.set(Value.kReverse);
  }

  public void toggle () {
    if (Objects.claw.get() == Value.kReverse) {
      Objects.claw.set(Value.kForward);
    } else {
      Objects.claw.set(Value.kReverse);
    }
  }

  public void spinSlow() {
    Objects.spinnerLeft.set(Constants.kSpeedIn / 4.0);
    Objects.spinnerRight.set(Constants.kSpeedIn / 4.0);
  }

  public void spinIn() {
    Objects.spinnerLeft.set(Constants.kSpeedIn);
    Objects.spinnerRight.set(Constants.kSpeedIn);
  }

  public void spinOut() {
    Objects.spinnerLeft.set(Constants.kSpeedOut);
    Objects.spinnerLeft.set(Constants.kSpeedOut);
  }

  public void spinOff() {
    Objects.spinnerLeft.set(0);
    Objects.spinnerRight.set(0);
  } 

  public void intake(GamePieces GP) {
    if(!(GP == GamePieces.Cone)) {
      openGrip();
      spinIn();
    } else {
      closeGrip();
      spinSlow();
    }
  }

  public boolean getSwitch() {
    return !Objects.IR_Sensor.get();
  }

  public void setMode(GamePieces mode) {
    if(mode != GamePieces.Cone) {
      openGrip();
    }
  }

  public void follow() {
    Objects.spinnerRight.follow(Objects.spinnerLeft);
  }

  public void periodic() {
    SmartDashboard.putBoolean("Intake/IR Sensor", !getSwitch());
    SmartDashboard.putString("Intake/Pneumatis", Objects.claw.get().toString());
    SmartDashboard.putNumber("Intake/Claw Spinner Left", Objects.spinnerLeft.get());
    SmartDashboard.putNumber("Intake/Claw Spinner Right", Objects.spinnerRight.get());
  }
}