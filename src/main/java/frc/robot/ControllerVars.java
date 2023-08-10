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


    
    // ControllerVars.substationPickupBtn
    //     .whileTrue(m_arm.moveToPositionCommand(positions.Substation))
    //     .onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    // ControllerVars.floorPickupBtn
    //     .whileTrue(m_arm.moveToPositionCommand(positions.Floor))
    //     .onFalse(m_claw.intakeCommand());

    // copilotController.button(2)
    //     .onTrue(new InstantCommand( () -> m_arm.goToScoreHigh().schedule()))
    //     .onFalse(m_arm.defaultCommand())
    //     .onFalse(m_claw.intakeCommand());

    // copilotController.button(3)
    //     .whileTrue(new InstantCommand( () -> m_arm.goToFloor().schedule()))
    //     .onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    // copilotController.button(4)
    //     .whileTrue(new InstantCommand( () -> m_arm.goToScoreMid().schedule()))
    //     .onFalse(m_claw.intakeCommand())
    //     .onFalse(m_arm.defaultCommand());
    
    // copilotController.button(5)
    //   .whileTrue(m_arm.moveToPositionCommand(positions.ScoreLow))
    //   .onFalse(m_claw.intakeCommand());
    
    // copilotController.button(6)
    //   .onTrue(new SequentialCommandGroup((m_claw.outTakeCommand()), new WaitCommand(0.25), m_arm.moveToPositionCommand(positions.Idle)))
    //   .onFalse(m_claw.spinOffCommand())
    //   .onFalse(m_claw.spinOffCommand());
    
    // copilotController.button(7).onTrue(m_LEDs.turnPurple().alongWith(new InstantCommand( () -> m_claw.setMode(GamePieces.Cube))).alongWith(new InstantCommand( () -> m_claw.setCone(false)).alongWith(new InstantCommand( () -> {copilotController.setLED(7, true);copilotController.setLED(8, false);}))));
    // copilotController.button(7).onTrue(new InstantCommand( () -> m_claw.setCone(false)));
    
    // copilotController.button(8).onTrue(m_LEDs.turnYellow().alongWith(new InstantCommand( () -> m_claw.setMode(GamePieces.Cone))).alongWith(new InstantCommand( () -> m_claw.setCone(true)).alongWith(new InstantCommand( () -> {copilotController.setLED(7, false);copilotController.setLED(8, true);}))));
    // copilotController.button(8).onTrue(new InstantCommand( () -> m_claw.setCone(true)));
    
    // copilotController.button(9).onTrue(m_arm.defaultCommand().alongWith(m_arm.onManual()));
    // copilotController.button(9).onFalse(m_arm.defaultCommand());
    
    // copilotController.button(12).onTrue(new InstantCommand( () -> {
    //   if (copilotController.getRawButton(9)) {
    //     m_claw.toggle(); 
    //   }
    // }));
    // copilotController.button(14).whileTrue(new InstantCommand( () -> {
    //   if (copilotController.getRawButton(9)) {
    //     m_claw.spinOut();
    //   }
    // }));
    // copilotController.button(14).onFalse(new InstantCommand( () -> {if (copilotController.getRawButton(9)) m_claw.spinOff();}));
    // copilotController.button(13).whileTrue(new InstantCommand( () -> {
    //   if (copilotController.getRawButton(9)) m_claw.spinIn();
      
    // }));
    // copilotController.button(13).onFalse(new InstantCommand( () -> {if (copilotController.getRawButton(9)) m_claw.spinOff();}));
}
