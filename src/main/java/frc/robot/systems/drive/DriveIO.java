package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.systems.drive.DriveVars.Objects;
import frc.robot.RobotStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveIO implements DriveIOInterface{

    public DriveIO() {
        SmartDashboard.putData("Field2d", Objects.fieldSim);
    }

    ///// TELEOP \\\\\
    public void swerveDrive(double x, double y, double rot, boolean field) {
        Objects.swerveDrive.drive(
            new Translation2d(x, y), 
            rot,
            field,
            RobotStates.sOpenLoop);
    }

    public void xLock() {
        Objects.swerveDrive.xLock();
    }

    public void toggleField() {
        RobotStates.sField = !RobotStates.sField;
    }

    public void zeroGyro() {
        Objects.gyro.setYaw(0);
    }

    ///// AUTON \\\\\
    public Pose2d getPose() {
        return Objects.swerveUtils.getPose();
    }

    public void resetPose(Pose2d pose) {
        Objects.swerveUtils.resetOdometry(pose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void autoBalance() {
        ProfiledPIDController balanceController = 
            new ProfiledPIDController(
                0.006, 0, 0.0016,
                new TrapezoidProfile.Constraints(1, 1));

        balanceController.setTolerance(45);

        swerveDrive(
            0, 
            balanceController
                .calculate(
                    Objects.gyro.getPitch(), 0),
            0, 
            false);
    }


    ///// Telemetry \\\\\
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState [] {
            Objects.modules[0].getState(),
            Objects.modules[1].getState(),
            Objects.modules[2].getState(),
            Objects.modules[3].getState()
        };

        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Objects.swerveDrive.getKinematics().toChassisSpeeds(getSwerveModuleStates());
    }

    public void telemetry() {
        moduleTelemetry();
    }

    public void moduleTelemetry() {
        for(int i = 0; i < Objects.modules.length; i++) {
            Objects.modules[i].setTelemetry(i);
        }

        SmartDashboard.putBoolean("Field Oriented", RobotStates.sField);
    }

    public void chassisTelemetry() {
        SmartDashboard.putNumber("Chassis Speeds X", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Y", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Rot", getChassisSpeeds().omegaRadiansPerSecond);

        SmartDashboard.putNumber("Gyro Yaw", Objects.gyro.getYaw());

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Degrees", getPose().getRotation().getDegrees());
    }

    public void putRobotOnField(Pose2d pose) {
        Objects.fieldSim.setRobotPose(pose);
    }

    public void update() {
        Objects.swerveUtils.updateOdometry();
    }

    public Command getAuton(String path, boolean useColor, Subsystem subsystem) {
        return Objects.swerveUtils.followPath(path, RobotStates.sEventMap, useColor, subsystem);
    }

    public void resetModules() {
        for(int i = 0; i < 4; i++) {
            Objects.modules[i].resetToAbsolute();
        }
    }
}