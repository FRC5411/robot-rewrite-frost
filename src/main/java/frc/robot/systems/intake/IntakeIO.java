// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.systems.intake.IntakeVars.Constants;
import frc.robot.systems.intake.IntakeVars.Objects;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotStates.*;

/** Add your docs here. */
public class IntakeIO {

  public IntakeIO() {}

  public void closeGrip() {
    Objects.kClaw.set(Value.kForward);
  }

  /** Open */
  public void openGrip() {
    Objects.kClaw.set(Value.kReverse);
  }

  public void toggle () {
    if (Objects.kClaw.get() == Value.kReverse) {
      Objects.kClaw.set(Value.kForward);
    } else {
      Objects.kClaw.set(Value.kReverse);
    }
  }

  public void spinSlow() {
    RobotStates.sIntakeSpeed = Constants.kSpeedIn/4;
  }

  public void spinIn() {
    RobotStates.sIntakeSpeed = Constants.kSpeedIn;
  }

  public void spinOut() {
    RobotStates.sIntakeSpeed = Constants.kSpeedOut;
  }

  public void spinOff() {
    RobotStates.sIntakeSpeed = 0;
  } 

  public void intake() {
    if(!RobotStates.sObjectState) {
      openGrip();
    } else {
      closeGrip();
    }
  }

  public void setCone(boolean check){
    RobotStates.sObjectState = check;
  }
  public boolean wantCone () {
    return RobotStates.sObjectState;
  }

  public void setMode(GamePieces mode) {
    RobotStates.sObjectState = (mode == GamePieces.Cone);
    spinSlow();

    if(!RobotStates.sObjectState){
      openGrip();
    }
  }

  public void periodic() {
    if ( DriverStation.isEnabled() || DriverStation.isAutonomousEnabled() ) {
      Objects.kSpinnerLeft.set(RobotStates.sIntakeSpeed);
      Objects.kSpinnerRight.set(RobotStates.sIntakeSpeed);

      // Fix once button boards get implemented
      if ( !Objects.kIR_Sensor.get() && (RobotContainer.getPosition() == positions.Substation || 
      RobotContainer.getPosition() == positions.Floor) && RobotStates.sObjectState) {
      //&& !RobotContainer.copilotController.getRawButton(15) ) {
        closeGrip();
      }
    } else {
      // prevent CAN timeouts when disabled, actual motor stoppage is handled at a lower level
      Objects.kSpinnerLeft.set(0);
      Objects.kSpinnerRight.set(0);
    }
}
}
