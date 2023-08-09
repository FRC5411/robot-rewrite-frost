package frc.robot.systems.drive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private SimpleMotorFeedforward ff;
    private PIDController azmthCont;
    private double modBoost = 15;
    private boolean useB = false;
    private double secoffset = 0;

    public FalconSwerveModule(WPI_TalonFX speed, WPI_TalonFX rotation, WPI_CANCoder encoder, double offset) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = encoder;

        lastAngle = new Rotation2d();

        Timer.delay(1);

        CTRESwerveConfigs.configDrive(
            Constants.kDriveKp, Constants.kDriveKf, 
            speed);

        CTRESwerveConfigs.configPosition(
            rot_encoder, offset);

        CTRESwerveConfigs.configAzimuth(
            Constants.kAzimuthKp, Constants.kAzimuthKd, Constants.kAzimuthKf, Constants.kAzimuthDeadBand,
            rotation, rot_encoder);

        debugState = new SwerveModuleState();

        ff = new SimpleMotorFeedforward(0.06, 0);

        azmthCont = new PIDController(Constants.kAzimuthKp, 0, Constants.kAzimuthKd);
        azmthCont.enableContinuousInput(0, 360);
        azmthCont.setTolerance(0);
    }

    public FalconSwerveModule(WPI_TalonFX speed, WPI_TalonFX rotation, WPI_CANCoder encoder, double offset, boolean invert, double secondoffset) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = encoder;

        lastAngle = new Rotation2d();

        Timer.delay(1);

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

        ff = new SimpleMotorFeedforward(0.06, 0);

        azmthCont = new PIDController(Constants.kAzimuthKp, 0, Constants.kAzimuthKd);
        azmthCont.enableContinuousInput(0, 360);
        azmthCont.setTolerance(0);

        secoffset = secondoffset;
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

        setDegrees(angle.getDegrees() + secoffset);

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
        double rotation = getEncoder().getAbsolutePosition();
        m_rotation.setSelectedSensorPosition(Conversions.degreesToFalcon(rotation, DriveVars.Constants.kAzimuthGearRatio));
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