package frc.robot.systems.drive;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.drive.DriveVars.Constants;
import frc.robot.utils.*;

public class FalconSwerveModule implements SwerveModuleInterface {
    private WPI_TalonFX m_speed;
    private WPI_TalonFX m_rotation;
    private WPI_CANCoder rot_encoder;
    private SwerveModuleState debugState;
    private Rotation2d lastAngle;
    private PIDController azmthCont;

    // If offset changes don't work, then remove offsets from CANCoder, and use subtraction in the getAngleRads() method
    public FalconSwerveModule(WPI_TalonFX speed, WPI_TalonFX rotation, WPI_CANCoder encoder, double offset) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = encoder;

        lastAngle = new Rotation2d();

        CTRESwerveConfigs.configPosition(
            rot_encoder, offset);

        CTRESwerveConfigs.configDrive(
            Constants.kDriveKp, Constants.kDriveKf,
            speed);

        CTRESwerveConfigs.configAzimuth(
            Constants.kAzimuthKp, Constants.kAzimuthKd, Constants.kAzimuthKf, Constants.kAzimuthDeadBand,
            rotation, rot_encoder);
        
        debugState = new SwerveModuleState();

        azmthCont = new PIDController(Constants.kAzimuthKp, 0, Constants.kAzimuthKd);
        azmthCont.setTolerance(0);

        Timer.delay(1);

        resetToAbsolute();
    }

    public FalconSwerveModule(WPI_TalonFX speed, WPI_TalonFX rotation, WPI_CANCoder encoder, double offset, boolean invert) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = encoder;

        lastAngle = new Rotation2d();

        CTRESwerveConfigs.configDrive(
            Constants.kDriveKp, Constants.kDriveKf, 
            speed);

            speed.setInverted(invert);

        CTRESwerveConfigs.configPosition(
            rot_encoder, offset);

        CTRESwerveConfigs.configAzimuth(
            Constants.kAzimuthKp, Constants.kAzimuthKd, Constants.kAzimuthKf, Constants.kAzimuthDeadBand,
            rotation, rot_encoder);

        debugState = new SwerveModuleState();

        azmthCont = new PIDController(Constants.kAzimuthKp, 0, Constants.kAzimuthKd);
        azmthCont.enableContinuousInput(0, 360);
        azmthCont.setTolerance(0);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        // This optimization method from WPI doesn't account for the 180 degree flip in the azimuth
        // And is the reason we decided to not use the regualr internal falcon controller
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngleRads());
        
        setDriveMPS(optimizedState, openLoop);
        setAngleDegrees(optimizedState);

        debugState = optimizedState;
    }

    @Override
    public void setDriveMPS(SwerveModuleState state, boolean openLoop) {
        double speed = state.speedMetersPerSecond;

        if (openLoop) {
            m_speed.set(speed/ 5.4);
        } else {
            m_speed.set(ControlMode.Velocity, 
            Conversions.MPSToFalcon(speed, Constants.kWheelPerimeterMeters, Constants.kDriveGearRatio));
        }
    }

    @Override
    public void setAngleDegrees(SwerveModuleState state) {
        //This line of code controls whether the swerve wheels go back 
        //into their starting position when the joystick are not being used, made by 364
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DriveVars.Constants.kMaxLinSpeedMeters * 0.01)) ? lastAngle : state.angle;
        //double angleDegrees = state.angle.getDegrees();

        setDegrees(angle.getDegrees());

        lastAngle = state.angle;
    }

    @Override
    public WPI_CANCoder getEncoder() {
        return rot_encoder;
    }

    @Override
    public double getDriveMeters() {
        return Conversions.falconToMeters(
            m_speed.getSelectedSensorPosition(), Constants.kWheelPerimeterMeters, Constants.kDriveGearRatio);
    }

    public double getDriveVelocity() {
        return Conversions.falconToMPS(
            m_speed.getSelectedSensorVelocity(), Constants.kWheelPerimeterMeters, Constants.kDriveGearRatio);
    }

    @Override
    public double getAnshulFactor() {
        return Constants.kScaleFactor;
    }

    @Override
    public Rotation2d getAngleRads() {
        return 
        new Rotation2d(Math.toRadians(getEncoder().getAbsolutePosition()));
    }

    @Override
    public void resetToAbsolute() {
        waitForCanCoder();
        double rotation = getEncoder().getAbsolutePosition();
        m_rotation.setSelectedSensorPosition(Conversions.degreesToFalcon(rotation, DriveVars.Constants.kAzimuthGearRatio));
    }

    private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        for (int i = 0; i < 100; ++i) {
            rot_encoder.getAbsolutePosition();
            if (rot_encoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);
        }
    }


    @Override
    public void resetToZero() {
        m_speed.setSelectedSensorPosition(0);
    }

    public void setDegrees(double angleDegrees) {
        m_rotation.set(ControlMode.PercentOutput, 
        azmthCont.calculate(
            Conversions.falconToDegrees(m_rotation.getSelectedSensorPosition(), 12.8) % 360,
            angleDegrees) 
        +
        0.06 * Math.signum(azmthCont.getPositionError()));
    }
    

    public SwerveModuleState getDesiredState() {
        return debugState;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            getAngleRads());
    }

    public void setTelemetry(int i) {
        SmartDashboard.putNumber("Module" + i + "/Degrees", getAngleRads().getDegrees());
        SmartDashboard.putNumber("Module" + i + "/Meters", getDriveMeters());
        SmartDashboard.putNumber("Module" + i + "/Velocity", getDriveVelocity());
        SmartDashboard.putNumber("Module" + i + "/Degrees Setpoint", getState().angle.getDegrees());
        SmartDashboard.putNumber("Module" + i + "/Velocity Setpoint", getState().speedMetersPerSecond);
    }
}