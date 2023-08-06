package frc.robot.systems.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveIOInterface {
    public void swerveDrive(double x, double y, double rot, boolean field);

    public void xLock();

    public void toggleField();

    public void zeroGyro();

    public Pose2d getPose();

    public void resetPose(Pose2d pose);

    public void resetPose();

    public void autoBalance();

    public SwerveModuleState[] getSwerveModuleStates();

    public ChassisSpeeds getChassisSpeeds();

    public void telemetry();

    public void moduleTelemetry();

    public void chassisTelemetry();

    public void putRobotOnField(Pose2d pose);

    public void update();

    public Command getAuton(String path, boolean useColor, Subsystem subsystem);
}