package frc.robot.systems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;

public class DriveSubsystem extends SubsystemBase {
    public DriveSimIO IO;

    public DriveSubsystem() {
        IO = new DriveSimIO();
    }

    public Command driveCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, BooleanSupplier field) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                IO.swerveDrive(x.getAsDouble(), y.getAsDouble(), z.getAsDouble(), field.getAsBoolean());
            },
            interrupted -> {}, 
            () -> false,
            this);
    }

    public Command autoBalanceCMD() {
      return new FunctionalCommand(
          () -> {},
          () -> {
            IO.autoBalance();
          },
          interrupted -> {IO.swerveDrive(0, 0, 0, false);}, 
          () -> false,
          this);
    }

    public Command resetPoseCMD(Pose2d pose) {
        return new InstantCommand(() -> IO.resetPose(pose), this);
    }

    public Command toggleFieldCMD() {
        return new InstantCommand(() -> IO.toggleField(), this);
    }

    public Command xLockCMD() {
        return new InstantCommand(() -> IO.xLock(), this);
    }

    public Command getAuton() {
        return IO.getAuton(RobotContainer.getAutonPath(), RobotStates.sUseColor, this);
    }

    @Override
    public void periodic() {
        IO.update();
        IO.telemetry();
        IO.putRobotOnField(IO.getPose());
    }
}