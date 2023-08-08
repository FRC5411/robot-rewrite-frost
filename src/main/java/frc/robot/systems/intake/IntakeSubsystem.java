// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIO IO;

  public IntakeSubsystem() {
    IO = new IntakeIO();
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> IO.intake(), this);
  }

  public Command outTakeCommand() {
    return new InstantCommand( () -> {
      if (RobotStates.sArmPosition == positions.Substation && IO.wantCone()) {
        IO.closeGrip();
      } else if ( IO.wantCone()) {
        if ( RobotStates.sArmPosition == positions.ScoreLow) {
          IO.spinOut();  
        }
        IO.spinOff();
        IO.openGrip();
      } else {
        IO.spinOut();
      }
    });
  }

  public void switchFunc(positions position) {
        switch (position) {
                    case ScoreHighCone:
                        break;
                    case ScoreHighCube:
                        break;
                    case ScoreMidCone:
                        break;
                    case ScoreMidCube:
                        break;
                    case ScoreLow:
                        break;
                    case Floor:
                        IO.spinIn();
                        IO.openGrip();
                        break;
                    case FloorAlt:
                        IO.spinIn();
                        IO.openGrip();
                        break;
                    case FloorAltCube:
                        IO.spinIn();
                        IO.openGrip();
                    case Substation:
                        IO.spinIn();
                        IO.openGrip();
                        break;
                    case Idle:
                        IO.spinSlow();
                        break;
                    default:
                        IO.spinSlow();
                        break;
  };
  }

  public Command defCommand(){
    return new FunctionalCommand(
    () -> {}, 
    () -> switchFunc(RobotStates.sArmPosition), 
    interrupted -> {}, 
    () -> false, 
    this);
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> IO.spinOff(), this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
