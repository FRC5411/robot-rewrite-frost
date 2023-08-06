package frc.robot.systems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.leds.LedVars.Constants;
import frc.robot.systems.leds.LedVars.Objects;

public class LedSubsytem extends SubsystemBase{
    LedIO IO;
    public LedSubsytem(){
        IO = new LedIO();
        Objects.ledStrip.setLength(Objects.ledBuffer.getLength());

        // Set the data
        Objects.ledStrip.setData(Objects.ledBuffer);
        Objects.ledStrip.start();
    }

    public Command turnPurple () { return new InstantCommand( () -> {
        IO.setColor((int)Math.floor(Color.kPurple.red*255), (int)Math.floor(Color.kPurple.green*255), (int)Math.floor(Color.kPurple.blue*255));
      }); 
    }


      public Command turnYellow () { return new InstantCommand( () -> {
        IO.setColor((int)Math.floor(Color.kOrange.red*255), (int)Math.floor(Color.kOrange.green*255), (int)Math.floor(Color.kOrange.blue*255));
      }); 
    }

      public Command flashGreen () { return new InstantCommand( () -> {
        Constants.k_r2 = Constants.k_r;
        Constants.k_g2 = Constants.k_g;
        Constants.k_b2 = Constants.k_b;
        IO.setColor(0, 255, 0);
      } ).andThen(new WaitCommand(0.25)).andThen(new InstantCommand( () -> {
        IO.setColor(Constants.k_r2, Constants.k_g2, Constants.k_b2);
      } )); 
    }
      
      @Override
      public void periodic() {
        for (var i = 0; i < Objects.ledBuffer.getLength(); i++) {
          Objects.ledBuffer.setRGB(i, Constants.k_r, Constants.k_g, Constants.k_b);
        }
    
        Objects.ledStrip.setData(Objects.ledBuffer);
        Objects.ledStrip.start();
      }
    
}
