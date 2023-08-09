package frc.robot.systems.drive;

import com.pathplanner.lib.PathConstraints;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.HolonomicDrive;
import edu.wpi.first.math.controller.PIDController;

public final class DriveVars {
    public static class Objects {
        public static final WPI_TalonFX LeftFront = new WPI_TalonFX(Constants.kFLDriveID, "drivetrain");
        public static final WPI_TalonFX RightFront = new WPI_TalonFX(Constants.kFRDriveID, "drivetrain");
        public static final WPI_TalonFX LeftBack = new WPI_TalonFX(Constants.kBLDriveID, "drivetrain");
        public static final WPI_TalonFX RightBack = new WPI_TalonFX(Constants.kBRDriveID, "drivetrain");

        public static final WPI_TalonFX rLeftFront = new WPI_TalonFX(Constants.kFLAzimuthID, "drivetrain");
        public static final WPI_TalonFX rRightFront = new WPI_TalonFX(Constants.kFRAzimuthID, "drivetrain");
        public static final WPI_TalonFX rLeftBack = new WPI_TalonFX(Constants.kBLAzimuthID, "drivetrain");
        public static final WPI_TalonFX rRightBack = new WPI_TalonFX(Constants.kBRAzimuthID, "drivetrain");

        public static final WPI_CANCoder LeftFrontEncoder = new WPI_CANCoder(Constants.kFRCanCoderID, "drivetrain");
        public static final WPI_CANCoder RightFrontEncoder = new WPI_CANCoder(Constants.kFRCanCoderID, "drivetrain");
        public static final WPI_CANCoder LeftBackEncoder = new WPI_CANCoder(Constants.kBLCanCoderID, "drivetrain");
        public static final WPI_CANCoder RightBackEncoder = new WPI_CANCoder(Constants.kBRCanCoderID, "drivetrain");

        public static final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.kPigeonID, "drivetrain");

        public static final FalconSwerveModule TopLeft = new FalconSwerveModule(
            LeftFront, rLeftFront, LeftFrontEncoder, Constants.kFLEncOffset, true, 15.0);
        public static final FalconSwerveModule TopRight = new FalconSwerveModule(
            RightFront, rRightFront, RightFrontEncoder ,Constants.kFREncOffset);
        public static final FalconSwerveModule BottomLeft = new FalconSwerveModule(
            LeftBack, rLeftBack, LeftBackEncoder, Constants.kBLEncOffset);
        public static final FalconSwerveModule BottomRight = new FalconSwerveModule(
            RightBack, rRightBack, RightBackEncoder, Constants.kBREncOffset);

        public static final FalconSwerveModule[] modules = 
            new FalconSwerveModule[] {TopLeft, TopRight, BottomLeft, BottomRight};

        public static final SwerveDriveKinematics kinematics = 
            SwerveUtils.createSquareKinematics(Constants.kRobotWidthMeters);

        public static final HolonomicDrive swerveDrive = 
            new HolonomicDrive(modules, gyro, kinematics, Constants.kMaxLinSpeedMeters);

        public static final PIDController tPID = 
            new PIDController(
                Constants.kTranslationKp,
                Constants.kTranslationKi,
                Constants.kTranslationKd);

        public static final PIDController rPID = 
            new PIDController(
                Constants.kRotationKp,
                Constants.kRotationKi,
                Constants.kRotationKd
            );

        public static final SwerveUtils swerveUtils = 
            new SwerveUtils(tPID, rPID, swerveDrive);

        public static final Field2d fieldSim = new Field2d();
    }

    public static class Constants {
        public static final double kRobotWidthMeters = 0.6858;

        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kWheelPerimeterMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveGearRatio = 6.75;
        public static final double kAzimuthGearRatio = 12.8;

        public static final double kFLEncOffset = -313.506+0.5 - 180;
        public static final double kFREncOffset = -69.082+0.5;
        public static final double kBLEncOffset = -45.791 + 180;
        public static final double kBREncOffset = -257.783;
        
        public static final double kMaxLinSpeedMeters = 5.4;
        public static final double kMaxRotMeters = Math.PI*2;
        public static final double kShwerveSpeedPercent = 0.1;
        public static final double kRotScaleFactor = 0.65;

        public static final double kAzimuthKp = 0.0105; //0.2;//0.0105;//0.0115//0.0125;//0.025 //0.05//0.1 //0.01 //0.0053 sds: 0.2; rylan: 0.65
        public static final double kAzimuthKd = 0.000265;//0.000265;//0.000275;//0.0003;//0.0004;//0.0005;//0.0006;//0.0006125;//0.0006125//0.000625//0.00065//0.0006;//0.00055//0.0005;//0.002//0.001//0.00075 //0.0005;//0.00025
        public static final double kAzimuthKf = 0.05;//0.05;//0.05
        public static final double kAzimuthDeadBand = 0.06;//0.1;//0.06;//0.075over slop;//0.1Over slop//0.05 under slop

        // calculated via JVN calculator
        public static final double kDriveKp = 0.088062; //0.04;//0.07;//0.06; //0.044057
        public static final double kDriveKf = 0.028998;//0.04//0.06; //0.028998

        // Factor to make odometry accurate
        public static final double kScaleFactor = 0.02 * kDriveGearRatio  * kDriveGearRatio;

        //Ids
        public static final int kPigeonID = 3;

        public static final int kFLCanCoderID = 4;
        public static final int kFRCanCoderID = 5;
        public static final int kBLCanCoderID = 6;
        public static final int kBRCanCoderID = 7;

        public static final int kFLDriveID = 11;
        public static final int kFRDriveID = 12;
        public static final int kBLDriveID = 13;
        public static final int kBRDriveID = 14;

        public static final int kFLAzimuthID = 21;
        public static final int kFRAzimuthID = 22;
        public static final int kBLAzimuthID = 23;
        public static final int kBRAzimuthID = 24;

        public static final double kTranslationKp = 3.75;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
        public static final double kTranslationKi = 0;
        public static final double kTranslationKd = 0;

        public static final double kTranslationTolerance = 0;
        public static final double KTranslationILower = 0;
        public static final double kTranslationIUpper = 0;

        public static final double kRotationKp = 6.25;//12.5;//15;//0.00005
        public static final double kRotationKi = 0;
        public static final double kRotationKd = 0;

        public static final double kRotationTolerance = 0;
        public static final double KRotationILower = 0;
        public static final double kRotationIUpper = 0;

        public static final PathConstraints kAlignConstraints = new PathConstraints(2, 1);

        public static double xSim = 0;
        public static double ySim = 0;
        public static double thetaSim = 0;
    }

    public static class Simulation {
        public static final FalconModuleSim TopLeftSim = new FalconModuleSim(
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kDriveGearRatio, 0.1),
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kAzimuthGearRatio, 1)
        );

        public static final FalconModuleSim TopRightSim = new FalconModuleSim(
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kDriveGearRatio, 0.1),
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kAzimuthGearRatio, 1)
        );

        public static final FalconModuleSim BottomLeftSim = new FalconModuleSim(
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kDriveGearRatio, 0.1),
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kAzimuthGearRatio, 1)
        );

        public static final FalconModuleSim BottomRightSim = new FalconModuleSim(
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kDriveGearRatio, 0.1),
            new FlywheelSim(DCMotor.getFalcon500(1), Constants.kAzimuthGearRatio, 1)
        );

        public static final FalconModuleSim[] modules = {
            TopLeftSim,
            TopRightSim,
            BottomLeftSim,
            BottomRightSim
        };

        public static final HolonomicDrive swerveDriveSim = new HolonomicDrive(
            modules,
            Objects.gyro,
            Objects.kinematics,
            Constants.kMaxLinSpeedMeters
        );

        public static final SwerveUtils swerveUtilsSim = 
            new SwerveUtils(Objects.tPID, Objects.rPID, swerveDriveSim);
    }
}