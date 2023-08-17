package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.REVConfigs;
import java.util.HashMap;



public class ArmVars {
    public static final class Constants {
        public static final double kJointAngleDeadzone = 0;
        public static final double kJointIntegratorMin = -0.25;
        public static final double kJointIntegratorMax = 0.25;
        public static final int kJointCurrentLimitAmps = 40;
    }

    public static class Objects {
        public static ArmJoint jointStageOne = new ArmJoint(1);
        public static ArmJoint jointStageTwo = new ArmJoint(2);
        public static ArmJoint jointStageThree = new ArmJoint(3);
    }

    public static class Sets {
        public static class stageOneJoint {
            public static final int kArmID = 31;
            public static final int kEncoderID = 7;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);

            public static final double kArmOffsetDeg = 203.0;
            public static final double kArmLength = 20.0; 

            // Feedforward
            public static final double kS = 0.04;
            public static final double kG = 1.1;
            public static final double kV = 0;
            public static final double kA = 0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final double kP = 0.0450 / 12.0;//0.0450;
            public static final double kI = 0.0001;
            public static final double kD = 0.0003;
            public static final double kTolerance = 3;
            public static final double kMaxVelocity = 800;
            public static final double kMaxAccleration = 220;

            public static final ProfiledPIDController kArmPID = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccleration)
            );

        }

        public static class stageTwoJoint{
            public static final int kArmID = 33;
            public static final int kEncoderID = 8;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);
            
            public static final double kArmOffsetDeg = 270.0;
            public static final double kArmLength = 29.5; 

            // Feedforward
            public static final double kS = 0.025;
            public static final double kG = 0.6;///0.75;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final double kP = 2 * (0.0057 / 12.0);
            public static final double kI = 0;
            public static final double kD = 0.004;
            public static final double kTolerance = 5.0;
            public static final double kMaxVelocity = 1000.0;
            public static final double kMaxAccleration = 340.0;

            public static final ProfiledPIDController kArmPID = new ProfiledPIDController(
                kP, kI, kD, 
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccleration)
            );

        } 
        
        public static class stageThreeJoint{
            public static final int kArmID = 34;
            public static final int kEncoderID = 9;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);
            
            public static final double kArmOffsetDeg = 108.0;
            public static final double kArmLength = 19.0; 

            // Feedforward
            public static final double kS = 0.022;
            public static final double kG = 0.0;
            public static final double kV = 0.004;
            public static final double kA = 0.0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final double kP = 0.0255;
            public static final double kI = 0;
            public static final double kD = 0.0005;
            public static final double kTolerance = 0.0;
            public static final double kMaxVelocity = 800.0;
            public static final double kMaxAccleration = 560.0;

            public static final ProfiledPIDController kArmPID = new ProfiledPIDController(
                kP, kI, kD, 
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccleration)
            );

        } 

        public static class armPositions{
            public static enum positions {
            ScoreHighCone,
            DipHighCone,
            ScoreHighCube,
            ScoreMidCone,
            DipMidCone,
            ScoreMidCube,
            ScoreLow,
            Floor,
            FloorAlt,
            FloorAltCube,
            Substation,
            Idle,
            IdleShootPosition
        };

        public static ArmPosition scoreHighConePosition  = new ArmPosition(227, 262, 145);
        public static ArmPosition dipHighConePosition    = new ArmPosition(217, 226, 136);
        public static ArmPosition scoreHighCubePosition  = new ArmPosition(133.5, 328, 160);
        public static ArmPosition scoreMidConePosition   = new ArmPosition(133.5, 306, 139);
        public static ArmPosition dipMidConePosition     = new ArmPosition(133.5, 282, 139);
        public static ArmPosition scoreMidCubePosition   = new ArmPosition(133.5, 344, 110);
        public static ArmPosition idlePosition           = new ArmPosition(133.5, 354, 39);
        public static ArmPosition scoreLowPosition       = new ArmPosition(133.5, 317, 55);
        public static ArmPosition floorPosition          = new ArmPosition(133.5, 232, 141);
        public static ArmPosition floorAltPosition       = new ArmPosition(133.5, 260, 38);
        public static ArmPosition floorAltCubePosition   = new ArmPosition(133.5, 255, 90);
        public static ArmPosition substationPosition     = new ArmPosition(133.5, 326, 107);
        public static HashMap<positions, ArmPosition> positionMap = new HashMap<positions, ArmPosition>();

        public static void setPositionMap(){
            positionMap.put(positions.ScoreHighCone, scoreHighConePosition);
            positionMap.put(positions.ScoreMidCone, scoreMidConePosition);
            positionMap.put(positions.ScoreHighCube, scoreHighCubePosition);
            positionMap.put(positions.ScoreMidCube, scoreMidCubePosition);
            positionMap.put(positions.ScoreLow, scoreLowPosition);
            positionMap.put(positions.Floor, floorPosition);
            positionMap.put(positions.FloorAlt, floorAltPosition);
            positionMap.put(positions.FloorAltCube, floorAltCubePosition);
            positionMap.put(positions.Substation, substationPosition);
            positionMap.put(positions.Idle, idlePosition);
            positionMap.put(positions.DipHighCone, dipHighConePosition);
            positionMap.put(positions.DipMidCone, dipMidConePosition);
        }
    }

    }
}