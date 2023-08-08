package frc.robot.utils;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.PIDController;

public class CTRESwerveConfigs {

    // Falonc Swerve Drive motor Configs, the PID vals can be determined using sys id in drivetrain analysis
    public static WPI_TalonFX configDrive(double kp, double kf, WPI_TalonFX driveMotor) {
        StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 60, 60, 0);
        driveMotor.configFactoryDefault();
        driveMotor.setInverted(TalonFXInvertType.CounterClockwise);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.config_kP(0, kp);
        driveMotor.config_kF(0, kf);
        driveMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);

        return driveMotor;
    }

    // Cancoder most be configed first before the azimuth motor
    // Try using the 0.2 P first and see if it works since we are using the internal PID loop
    public static WPI_TalonFX configAzimuth (
      double kp, double kd, double kf, double deadband, WPI_TalonFX motor, CANCoder position) {
      
        StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 20, 20, 0);
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configRemoteFeedbackFilter(position, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
        motor.setSelectedSensorPosition(position.getAbsolutePosition());
        motor.config_kP(0, kp);
        motor.config_kD(0, kd);
        motor.config_kF(0, kf);
        motor.configNeutralDeadband(deadband);

        return motor;
      }

      // Config position
      public static CANCoder configPosition (CANCoder encoder, double offset) {
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.setPositionToAbsolute();

        return encoder;
      }

      public static PIDController configPID(double tolerance, double IUpperBound, double ILowerBound, PIDController PID) {
        PID.setTolerance(tolerance);
        PID.setIntegratorRange(ILowerBound, IUpperBound);
        return PID;
      }

}
