package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.systems.drive.DriveVars.Objects;
import frc.robot.systems.drive.DriveVars.Simulation;
import frc.robot.RobotStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSimIO implements DriveIOInterface{

    public DriveSimIO() {
        SmartDashboard.putData("Field2d", Objects.fieldSim);
    }

    ///// TELEOP \\\\\
    public void swerveDrive(double x, double y, double rot, boolean field) {
        Simulation.swerveDriveSim.drive(
            new Translation2d(x, y), 
            rot,
            field,
            RobotStates.sOpenLoop);
    }

    public void xLock() {
        Simulation.swerveDriveSim.xLock();
    }

    public void toggleField() {
        RobotStates.sField = !RobotStates.sField;
    }

    public void zeroGyro() {}

    ///// AUTON \\\\\
    public Pose2d getPose() {
        return Simulation.swerveUtilsSim.getPose();
    }

    public void resetPose(Pose2d pose) {
        Simulation.swerveUtilsSim.resetOdometry(pose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void autoBalance() {}

    ///// Telemetry \\\\\
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState [] {
            Simulation.modules[0].getState(),
            Simulation.modules[1].getState(),
            Simulation.modules[2].getState(),
            Simulation.modules[3].getState()
        };

        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Simulation.swerveDriveSim.getKinematics().toChassisSpeeds(getSwerveModuleStates());
    }

    public void telemetry() {
        moduleTelemetry();
        chassisTelemetry();
    }

    public void moduleTelemetry() {
        for(int i = 0; i < Simulation.modules.length; i++) {
            Simulation.modules[i].setTelemetry(i);
        }

        SmartDashboard.putBoolean("Field Oriented", RobotStates.sField);
    }

    public void chassisTelemetry() {
        SmartDashboard.putNumber("Chassis Speeds X", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Y", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Rot", getChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Degrees", getPose().getRotation().getDegrees());
    }

    public void putRobotOnField(Pose2d pose) {
        Objects.fieldSim.setRobotPose(pose);
    }

    public void update() {
        for(int i = 0; i < Simulation.modules.length ; i++) {
            Simulation.modules[i].update(0.2);
        }

        Simulation.swerveUtilsSim.updateOdometry();
    }

    public Command getAuton(String path, boolean useColor, Subsystem subsystem) {
        return Simulation.swerveUtilsSim.followPath(path, RobotStates.sEventMap, useColor, subsystem);
    }
}