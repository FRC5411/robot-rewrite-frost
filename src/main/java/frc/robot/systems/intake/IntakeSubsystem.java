package frc.robot.systems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import frc.robot.systems.intake.IntakeVars.GamePieces;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIO IO;

  public IntakeSubsystem() {
    IO = new IntakeIO();
    IO.follow();
    IO.spinSlow();
  }

  public Command manualIntakeCommand(GamePieces GP) {
    return new InstantCommand(() -> IO.intake(GP), this).repeatedly().withTimeout(1.5);
  }

  public Command outTakeCommand(positions pos, GamePieces GP) {
    return new InstantCommand( () -> {
      if (pos == positions.Substation && (GP == GamePieces.Cone)) {
        IO.closeGrip();
      } else if ( GP == GamePieces.Cone) {
        if ( pos == positions.ScoreLow) {
          IO.spinOut();  
        }
        IO.spinSlow();
        IO.openGrip();
      } else {
        IO.spinOut();
      }
    }).repeatedly().withTimeout(1.5);
  }

  public Command setMode(GamePieces GP) {
    return new InstantCommand(() -> IO.setMode(GP));
  }

  public Command detectIntakeCommand(BooleanSupplier detect, BooleanSupplier end) {
    return new FunctionalCommand(
      () -> {},
      () -> {if(detect.getAsBoolean()) {IO.closeGrip();}}, 
      (interrupted) -> {}, 
      detect, 
      this);
  }

  public IntakeIO getIO() {
    return IO;
  }

  public void setNewIntakePos(positions position) {
    if(position != positions.Idle){
      IO.openGrip();
      IO.spinIn(); 
    } else {
      IO.spinSlow();
    }
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> IO.spinOff(), this);
  }

  public Command spinOutCommand() {
    return new InstantCommand(() -> IO.spinOut(), this);
  }

  public Command toggleCMD() {
    return new InstantCommand(() -> IO.toggle());
  }

  public Command DEFspinSlowCommand() {
    return new FunctionalCommand(
      () -> {}, 
      () -> IO.spinSlow(),
      interrupted -> {}, 
      () -> false, 
      this);
  }

  @Override
  public void periodic() {
    IO.periodic();
  }
}
