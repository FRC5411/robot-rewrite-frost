package frc.robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.ButtonBoard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerVars {
    public static final CommandXboxController xboxController = new CommandXboxController(0);
    public static final ButtonBoard copilotController = new ButtonBoard(1, 2);

    public static final Trigger resetOdometryBtn = xboxController.a();
    public static final Trigger toggleRobotOrientBtn = xboxController.b();
    // public static final Trigger engageLimeLightBtn = xboxController.x();
    public static final Trigger engageAutobalanceBtn = xboxController.y();
    
    ///////////////////////////////////////////////
    // BUTTON BOARD
    public static final Trigger substationPickupBtn = copilotController.button(0);
    public static final Trigger floorPickupBtn = copilotController.button(1);
    public static final Trigger scoreHighBtn = copilotController.button(2);
    public static final Trigger altFloorPickupBtn = copilotController.button(3);
    public static final Trigger scoreMidBtn = copilotController.button(4);
    public static final Trigger scoreLowBtn = copilotController.button(5);
    public static final Trigger placeIdleBtn = copilotController.button(6);
    public static final Trigger coneModeBtn = copilotController.button(7);
    public static final Trigger cubeModeBtn = copilotController.button(8);
}
