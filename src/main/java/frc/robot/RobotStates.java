package frc.robot;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.arm.ArmVars.Sets.armPositions;

public class RobotStates {
    public static boolean sField = false;
    public static boolean sOpenLoop = true;
    public static double sIntakeSpeed = 0;
    public static boolean sObjectState = true;
    public static boolean sUseColor = true;
    public static String sAutonPath = "New Path";

    public static HashMap<String, Command> sEventMap = new HashMap<String, Command>();

    public enum GamePieces {
        Cone,
        Cube,
        None
    }

    public static armPositions.positions sArmPosition = armPositions.positions.Idle;
}