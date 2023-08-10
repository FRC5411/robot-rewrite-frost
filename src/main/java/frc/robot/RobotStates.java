package frc.robot;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotStates {
    public static boolean sField = false;
    public static boolean sOpenLoop = true;
    public static boolean sUseColor = true;
    public static String sAutonPath = "New Path";

    public static HashMap<String, Command> sEventMap = new HashMap<String, Command>();
}