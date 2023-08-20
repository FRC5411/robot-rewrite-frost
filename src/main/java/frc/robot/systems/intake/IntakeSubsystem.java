package frc.robot.systems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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

  public Command detectIntakeCommand(GamePieces GP, BooleanSupplier detect, BooleanSupplier end) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        if(detect.getAsBoolean()) {
          IO.intake(GP);
        }}, 
      (interrupted) -> {}, 
      detect, 
      this);
  }

  public Command intakeCommand(GamePieces GP, BooleanSupplier detect, BooleanSupplier end) {
    return new ScheduleCommand(
        detectIntakeCommand(
          GP, 
          detect, 
          end));
  }

  public IntakeIO getIO() {
    return IO;
  }

  public void setNewIntakePos(positions position) {
    switch (position) {
        case DipHighCone:
        case ScoreHighCone:
        case ScoreHighCube:
        case DipMidCone:
        case ScoreMidCone:
        case ScoreMidCube:
        case ScoreLow:
            break;
        case Floor:
        case FloorAlt:
        case Substation:
            IO.openGrip();
            IO.spinIn();
        break;
        case Idle:
            IO.spinSlow();
        break;
        case IdleShootPosition:
            IO.spinOut();
        break;
        default:
            IO.spinSlow();
        break;
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
