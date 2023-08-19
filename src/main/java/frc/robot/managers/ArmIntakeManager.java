package frc.robot.managers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.systems.arm.ArmPosition;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.arm.ArmVars.Sets.armPositions;
import frc.robot.systems.arm.ArmVars.Sets.armPositions.positions;

import frc.robot.systems.intake.IntakeVars.GamePieces;
import frc.robot.systems.intake.IntakeSubsystem;

import frc.robot.systems.leds.LedSubsytem;
import frc.robot.ControllerVars;
import edu.wpi.first.wpilibj.DriverStation;

public class ArmIntakeManager {
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    LedSubsytem LEDs;

    positions mPos;
    GamePieces mGP;

    public ArmIntakeManager (ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, LedSubsytem LEDs) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.LEDs = LEDs;

        mPos = positions.Idle;
        mGP = GamePieces.Cone;
    }


    public void scheduleDefaultCMD(boolean isDip) {
        if(ControllerVars.copilotController.isButtonDown(9)) {
          armSubsystem.manualUpdateCMD(0, 0, 0).schedule();
        } else {
          if(DriverStation.isAutonomous()) return;
          
          if(isDip) goToIdleDip().schedule();
          else goToIdle().schedule();
        }
      }

    
    ///// ALL COMMANDS \\\\\\
    public Command setMode(GamePieces GP) {
        return new InstantCommand(() -> {
            mGP = GP;
        }).alongWith(updateLEDs());
    }

    public Command outTakeCommand() {
        return intakeSubsystem.outTakeCommand(mPos, mGP);
    }

    public Command manualIntakeCommand() {
        return intakeSubsystem.manualIntakeCommand(mGP);
    }

    public Command detectIntakeCommand() {
        return intakeSubsystem.detectIntakeCommand(
            () -> intakeSubsystem.getIO().getSwitch(), 
            () -> (mGP == GamePieces.Cone) || (mPos == positions.Substation) || (mPos == positions.FloorAlt) || (mPos == positions.Floor));
    }

    public Command gotToPickUpAltCube() {
        return goToPosition(positions.FloorAltCube).andThen(updateLEDs());
    }

    public Command goToPickUpAltCone() {
        return goToPosition(positions.FloorAlt).andThen(updateLEDs());
    }

    public Command goToPickUpAlt() {
        if(mGP == GamePieces.Cone) {
            return goToPickUpAltCone();
        } else {
            return gotToPickUpAltCube();
        }
    }

    public Command goToPickup() {
        return goToPosition(positions.Floor);
    }

    public Command goToLowScore() {
            return goToPosition(positions.ScoreLow);
    }

    public Command goToIdle() {
        return goToPosition(positions.Idle);
    }

    public Command goToIdleDip() {
        SequentialCommandGroup armCmd = 
            new SequentialCommandGroup(
                updateArmAndIntakeDipReturn(positions.Idle),
                new WaitCommand(1.0),
                closeIntake(),
                updateArmAndIntake(positions.Idle, positions.Idle) // Going back to idle
                ); 
        return armCmd;
    }

    public Command goToMidScore() {
        if (mGP == GamePieces.Cone) {
            return goToPositionDip(positions.ScoreMidCone, 0.6, positions.DipMidCone);
        } else {
            return goToPosition(positions.ScoreMidCube);
        }
    }

    public Command goToSubstation() {
        return goToPosition(positions.Substation);
    }

    public Command goToHighScore() {
        if (mGP == GamePieces.Cone) {
            return goToPositionDip(positions.ScoreHighCone, 0.6, positions.DipHighCone);
        } else {
            return goToPosition(positions.ScoreHighCube);
        }
    }

    public Command goToPositionDip(positions pos1, double wait, positions pos2) {
        SequentialCommandGroup armCmd = 
            new SequentialCommandGroup(
                updateArmAndIntake(pos1, positions.Idle),
                new WaitCommand(wait),
                updateArmAndIntake(pos2, pos2));
        return armCmd;
    }

    public Command goToPosition(positions pos1) {
        return updateArmAndIntake(pos1, pos1);
    }


    public Command updateArmAndIntake(positions selectedPos, positions idle) {
        mPos = selectedPos;
        return new InstantCommand(
            () -> {
                armSubsystem.updateSetpoints(selectedPos, false);
                if(selectedPos == positions.FloorAlt || selectedPos == positions.Floor || selectedPos == positions.Substation)
                    intakeSubsystem.setNewIntakePos(idle);
            }
        );
    }

    public Command closeIntake(){
        return new InstantCommand(
            () -> {
                if(mGP == GamePieces.Cone)
                intakeSubsystem.getIO().closeGrip();
            }
        );
    }

    public Command updateArmAndIntakeDipReturn(positions selectedPos) {
        mPos = selectedPos;
        return new InstantCommand(
            () -> {
                armSubsystem.updateSetpoints(selectedPos, true);
            }
        );
    }   

    public Command updateLEDs() {
        if(mGP == GamePieces.Cube) {
            return LEDs.turnPurple();
        }
        if(mGP == GamePieces.Cone) {
            return LEDs.turnYellow();
        }
        return LEDs.turnOff();
    }

    ///// GETTERS \\\\\\
    public ArmPosition getPos() {
        SmartDashboard.putString("Pos", mPos.toString());

        ArmPosition pos = armPositions.positionMap.get(mPos);
        return pos;
    }

    public GamePieces getGP() {
        return mGP;
    }
}