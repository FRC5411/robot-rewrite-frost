// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.RobotStates.*;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIO IO;

  public IntakeSubsystem() {
    IO = new IntakeIO();
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> IO.intake(), this);
  }

  // WILL NEED PROPER INPUT FOR THE BOOLEAN
  public Command outTakeCommand() {
    return new InstantCommand( () -> {
      if (RobotContainer.getPosition() == positions.Substation && IO.wantCone()) {
        IO.closeGrip();
      } else if ( IO.wantCone()) {
        if ( RobotContainer.getPosition() == positions.ScoreLow) {
          IO.spinOut();  
        }
        IO.spinOff();
        IO.openGrip();
      } else {
        IO.spinOut();
      }
    });
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> IO.spinOff(), this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
