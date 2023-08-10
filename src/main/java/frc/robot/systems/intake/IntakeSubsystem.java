package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIO IO;

  public IntakeSubsystem() {
    IO = new IntakeIO();
  }

  public Command manualIntakeCommand() {
    return new InstantCommand(() -> IO.intake(), this);
  }

  public Command outTakeCommand(positions pos) {
    return new InstantCommand( () -> {
      if (pos == positions.Substation && IO.wantCone()) {
        IO.closeGrip();
      } else if ( IO.wantCone()) {
        if ( pos == positions.ScoreLow) {
          IO.spinOut();  
        }
        IO.spinOff();
        IO.openGrip();
      } else {
        IO.spinOut();
      }
    });
  }

  public Command commandChooser(positions position) {
        switch (position) {
                    case ScoreHighCone:
                    case ScoreHighCube:
                    case ScoreMidCone:
                    case ScoreMidCube:
                    case ScoreLow:
                    case Floor:
                    case FloorAlt:
                    case FloorAltCube:
                    case Substation:
                        return scheduleSpinSlow();
                    case Idle:
                    default:
                        return schedulePickUp();
      }
  }

  public Command scheduleSpinSlow() {
    Command defCommand = new InstantCommand(() -> IO.spinSlow());
    return defCommand;
  }

  public Command schedulePickUp() {
    Command command = new InstantCommand(() -> {IO.spinIn(); IO.openGrip();});
    return command;
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> IO.spinOff(), this);
  }


  @Override
  public void periodic() {}
}
