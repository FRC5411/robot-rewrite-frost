// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */
  public IntakeSubsystem() {}

  public Command intakeCommand() {
    return new InstantCommand(() -> intake(), this);
  }

  public Command outTakeCommand() {
    return new InstantCommand( () -> {
      if (m_container.getArm().target == positions.Substation && m_cone) {
        closeGrip();
      } else if ( m_cone ) {
        if ( m_container.getArm().target == positions.ScoreLow) {
          spinOut();  
        }
        spinOff();
        openGrip();
      } else {
        spinOut();
      }
    });
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> spinOff(), this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
