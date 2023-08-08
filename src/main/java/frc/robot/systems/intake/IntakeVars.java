// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import com.revrobotics.CANSparkMax;
import frc.robot.utils.REVConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */
public class IntakeVars {
    public static final class Constants {
         public static final double kSpeedOut = 0.8;
         public static final double kSpeedIn = 0.5;

         public static final int kForwardChannel = 14;
         public static final int kBackwardChannel = 15;
         public static final int kGripLeftID =  41;
         public static final int kGripRightID =  42;
         public static final int kPCH_ID = 1;
         public static final int kIR_Sensor_ID = 1;
    }

    public static class Objects {
        public static DoubleSolenoid claw = new DoubleSolenoid(Constants.kPCH_ID, PneumaticsModuleType.REVPH, Constants.kForwardChannel, Constants.kBackwardChannel);
        public static CANSparkMax spinnerLeft = REVConfigs.initNEO550Motor(Constants.kGripLeftID, false);
        public static CANSparkMax spinnerRight = REVConfigs.initNEO550Motor(Constants.kGripRightID, false);
        public static DigitalInput IR_Sensor = new DigitalInput(Constants.kIR_Sensor_ID);
    }
}

