package frc.robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerVars {
    public static final CommandXboxController xboxController = new CommandXboxController(0);

    public static final Trigger a = xboxController.a();
    public static final Trigger b = xboxController.b();
    public static final Trigger x = xboxController.x();
    public static final Trigger y = xboxController.y();
}
