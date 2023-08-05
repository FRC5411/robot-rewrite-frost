package frc.robot.systems.drive;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.SwerveModuleInterface;

public class FalconModuleSim implements SwerveModuleInterface{
    private FlywheelSim driveMotorSim;
    private FlywheelSim turnMotorSim;
    private double drivePosMeters = 0;
    private double turnPosRadians = 0;
    private PIDController turnPIDController;
    private PIDController drivePIDController;
    private SwerveModuleState debugState = new SwerveModuleState();

    public FalconModuleSim(FlywheelSim driveMotorSim, FlywheelSim turnMotorSim) {
        this.driveMotorSim = driveMotorSim;
        this.turnMotorSim = turnMotorSim;
        this.drivePIDController = 
            new PIDController(DriveVars.Constants.kDriveKp, 0.0, 0.0);
        this.turnPIDController = 
            new PIDController(DriveVars.Constants.kAzimuthKp, 0.0, DriveVars.Constants.kAzimuthKd);

        this.turnPIDController.enableContinuousInput(0, 360);

        this.drivePIDController.setTolerance(0.0);
        this.turnPIDController.setTolerance(0.0);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        setDriveMPS(state, openLoop);
        setAngleDegrees(state);

        debugState = state;
    }

    @Override
    public void setDriveMPS(SwerveModuleState state, boolean openLoop) {
        if(openLoop) {
            setDriveVoltage(
                (state.speedMetersPerSecond / DriveVars.Constants.kMaxLinSpeedMeters) * 12);
        } else {
            double driveOutput = drivePIDController.calculate(getDriveVelocityMetersPerSec(), state.speedMetersPerSecond);
            setDriveVoltage(driveOutput);
        }
    }

    @Override
    public void setAngleDegrees(SwerveModuleState state) {
        double turnOutput = turnPIDController.calculate(getAngleRads().getDegrees(), state.angle.getDegrees());
        setTurnVoltage(turnOutput);
    }

    @Override
    public Rotation2d getAngleRads() {
        return new Rotation2d(turnPosRadians);
    }

    @Override
    public double getDriveMeters() {
        return drivePosMeters;
    }

    @Override
    public double getAnshulFactor() {
        return 1.0;
    }

    @Override
    public WPI_CANCoder getEncoder() {
        return new WPI_CANCoder(0);
    }

    @Override
    public void resetToAbsolute() {}

    @Override
    public void resetToZero() {
        drivePosMeters = 0.0;
        turnPosRadians = 0.0;
    }
        
    public void setDriveVoltage(double voltage) {
        driveMotorSim.setInputVoltage(voltage);
    }

    public void setTurnVoltage(double voltage) {
        turnMotorSim.setInputVoltage(voltage);
    }

    public void update(double dt) {
        driveMotorSim.update(dt);
        turnMotorSim.update(dt);

        integrateSpeeds();
    }

    public void integrateSpeeds() {
        drivePosMeters += toLinearVelocity(driveMotorSim.getAngularVelocityRadPerSec()) * 0.02;
        turnPosRadians += turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
    }

    public double toLinearVelocity(double radians) {
        return radians * DriveVars.Constants.kWheelDiameterMeters / 2.0;
    }

    public double getDriveVelocityMetersPerSec() {
        return toLinearVelocity(driveMotorSim.getAngularVelocityRadPerSec());
    }

    public double getTurnVelocityRadiansPerSec() {
        return turnMotorSim.getAngularVelocityRadPerSec();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), new Rotation2d(turnPosRadians));
    }

    public SwerveModuleState getDesiredState() {
        return debugState;
    }

    public void setTelemetry(int i) {
        SmartDashboard.putNumber("Module" + i + "/Degrees", getAngleRads().getDegrees());
        SmartDashboard.putNumber("Module" + i + "/Meters", getDriveMeters());
        SmartDashboard.putNumber("Module" + i + "/Velocity", getDriveVelocityMetersPerSec());
        SmartDashboard.putNumber("Module" + i + "/Degrees Setpoint", getState().angle.getDegrees());
        SmartDashboard.putNumber("Module" + i + "/Velocity Setpoint", getState().speedMetersPerSecond);
    }
}