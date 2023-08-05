package frc.robot.systems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.systems.drive.DriveVars.Objects;
import frc.robot.RobotStates;

public class DriveSubsystem extends SubsystemBase {
    public DriveIOInterface driveIO;

    public DriveSubsystem() {
        driveIO = new DriveSimIO();
    }

    public Command driveCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, BooleanSupplier field) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                driveIO.swerveDrive(x.getAsDouble(), y.getAsDouble(), z.getAsDouble(), field.getAsBoolean());
            },
            interrupted -> {}, 
            () -> false,
            this);
    }

    public Command autoBalanceCMD() {
      return new FunctionalCommand(
          () -> {},
          () -> {
            driveIO.autoBalance();
          },
          interrupted -> {}, 
          () -> false,
          this);
    }

    public Command resetPoseCMD(Pose2d pose) {
        return new InstantCommand(() -> driveIO.resetPose(pose), this);
    }

    public Command toggleFieldCMD() {
        return new InstantCommand(() -> driveIO.toggleField(), this);
    }

    public Command xLockCMD() {
        return new InstantCommand(() -> driveIO.xLock(), this);
    }

    public Command getAuton() {
        return Objects.swerveUtils.followPath("New Path", RobotStates.sEventMap, true, this);
    }

    @Override
    public void periodic() {
        Objects.swerveUtils.updateOdometry();
        driveIO.telemetry();
        driveIO.putRobotOnField(driveIO.getPose());
    }
}