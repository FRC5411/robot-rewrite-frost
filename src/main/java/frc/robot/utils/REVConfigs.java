package frc.robot.utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class REVConfigs {

    // Falonc Swerve Drive motor Configs, the PID vals can be determined using sys id in drivetrain analysis
    public static CANSparkMax initNEO550Motor(int motorId, boolean invert) {
        CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(20);
        motor.setInverted(invert);
        motor.setCANTimeout(20);
        motor.burnFlash();
        
        return motor;
    }

    public static void NEO550(CANSparkMax motor, boolean invert) {
        motor.restoreFactoryDefaults();
        // motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(invert);
    }

    public static CANSparkMax initNEOMotor(int motorId, boolean isInverted) {
        CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);
        
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(isInverted);
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.burnFlash();

        return motor;
    }
}
