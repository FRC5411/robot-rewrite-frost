package frc.robot.managers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;
import frc.robot.systems.intake.IntakeSubsystem;

public class ArmIntakeManager {
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    positions mPos;

    public ArmIntakeManager (ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        mPos = positions.Idle;
    }

    public Command gotToPickUpAltCube() {
        return goToPosition(positions.FloorAltCube);
    }

    public Command goToPickUpAltCone() {
        return goToPosition(positions.FloorAlt);
    }

    public Command goToPickup() {
        return goToPosition(positions.Floor);
    }

    public Command goToLowScore(boolean cone) {
            return goToPosition(positions.ScoreLow);
    }

    public Command goToIdle() {
        return goToPosition(positions.Idle);
    }

    public Command goToMidScore(boolean cone) {
        if (cone) {
            return goToPositionDip(positions.ScoreMidCone, 1.5, positions.DipMidCone);
        } else {
            return goToPosition(positions.ScoreMidCube);
        }
    }

    public Command goToSubstation() {
        return goToPosition(positions.Substation);
    }

    public Command goToHighCone(boolean cone) {
        if (cone) {
            return goToPositionDip(positions.ScoreHighCone, 1.5, positions.DipHighCone);
        } else {
            return goToPosition(positions.ScoreHighCube);
        }
    }

    public Command goToPositionDip(positions pos1, double wait, positions pos2) {
        mPos = pos1;
        SequentialCommandGroup armCmdd = 
            new SequentialCommandGroup(
                upDateSystems(),
                new WaitCommand(wait),
                new InstantCommand(() -> mPos = pos2),
                upDateSystems());
        return armCmdd;
    }

    public Command goToPosition(positions pos1) {
        mPos = pos1;
        return upDateSystems();
    }

    public Command upDateSystems() {
        return new InstantCommand(() -> {
            intakeSubsystem.commandChooser(mPos);
            armSubsystem.updateSetPointsCMD(mPos);
        });
    }
}