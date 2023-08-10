package frc.robot.systems.intake;
import frc.robot.systems.intake.IntakeVars.Constants;
import frc.robot.systems.intake.IntakeVars.GamePieces;
import frc.robot.systems.intake.IntakeVars.Objects;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeIO {

  public IntakeIO() {}

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
  }

  public void spinIn() {
    Objects.spinnerLeft.set(Constants.kSpeedIn);
  }

  public void spinOut() {
    Objects.spinnerLeft.set(Constants.kSpeedOut);
  }

  public void spinOff() {
    Objects.spinnerLeft.set(0);
  } 

  public void intake(GamePieces GP) {
    if(!(GP == GamePieces.Cone)) {
      openGrip();
      spinIn();
    } else {
      closeGrip();
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

  public void periodic() {}
}