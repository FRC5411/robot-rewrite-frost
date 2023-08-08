package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * Yes, buttons start at 0, i subtract because LabVIEW
 */

public class ButtonBoard {
    CommandGenericHID leftBoard;
    CommandGenericHID rightBoard;

    public ButtonBoard (int leftBoardPort, int rightBoardPort) {
        leftBoard = new CommandGenericHID(leftBoardPort);
        rightBoard = new CommandGenericHID(rightBoardPort);
    }

    public Trigger button ( int buttonNumber ) {
        return (buttonNumber < 9) // Is button single digit?
            ? leftBoard.button(buttonNumber + 1)
            : rightBoard.button(buttonNumber - 8);
    }

    public boolean isButtonDown ( int buttonNumber ) {
        return (buttonNumber < 9) // Is button single digit?
            ? leftBoard.getHID().getRawButton(buttonNumber + 1) 
            : rightBoard.getHID().getRawButton(buttonNumber - 8);
    }

    private double getAxis ( int axisNumber ) {
        if( Math.abs(rightBoard.getRawAxis(axisNumber)) < 0.5 ) return 0;
        return rightBoard.getRawAxis(axisNumber);
    }

    public Translation2d getJoystick () {
        return new Translation2d(getAxis(0), -getAxis(1));
    }
}