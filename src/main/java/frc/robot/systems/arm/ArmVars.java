// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
        public static class stageOneJoint{
            public static final int kArmID = 1;
            public static final int kEncoderID = 1;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);

            public static final double kArmOffsetDeg = 0;
            public static final double kArmLength = 0; 

            // Feedforward
            public static final int kS = 0;
            public static final int kG = 0;
            public static final int kV = 0;
            public static final int kA = 0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final int kP = 0;
            public static final int kI = 0;
            public static final int kD = 0;
            public static final int kTolerance = 0;
            public static final int kMaxVelocity = 0;
            public static final int kMaxAccleration = 0;

            public static final ProfiledPIDController kArmPID = new ProfiledPIDController(
                kP, kI, kD, 
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccleration)
            );

        }

        public static class stageTwoJoint{
            public static final int kArmID = 1;
            public static final int kEncoderID = 1;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);
            
            public static final double kArmOffsetDeg = 0;
            public static final double kArmLength = 0; 

            // Feedforward
            public static final int kS = 0;
            public static final int kG = 0;
            public static final int kV = 0;
            public static final int kA = 0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final int kP = 0;
            public static final int kI = 0;
            public static final int kD = 0;
            public static final int kTolerance = 0;
            public static final int kMaxVelocity = 0;
            public static final int kMaxAccleration = 0;

            public static final ProfiledPIDController kArmPID = new ProfiledPIDController(
                kP, kI, kD, 
                new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccleration)
            );

        } 
        
        public static class stageThreeJoint{
            public static final int kArmID = 1;
            public static final int kEncoderID = 1;
            public static final CANSparkMax kArmMotor = REVConfigs.initNEOMotor(kArmID, false);
            public static final DutyCycleEncoder kArmEncoder = new DutyCycleEncoder(kEncoderID);
            
            public static final double kArmOffsetDeg = 0;
            public static final double kArmLength = 0; 

            // Feedforward
            public static final int kS = 0;
            public static final int kG = 0;
            public static final int kV = 0;
            public static final int kA = 0;

            public static final ArmFeedforward kArmFF = new ArmFeedforward(kS, kG, kV, kA);
            
            // PID
            public static final int kP = 0;
            public static final int kI = 0;
            public static final int kD = 0;
            public static final int kTolerance = 0;
            public static final int kMaxVelocity = 0;
            public static final int kMaxAccleration = 0;

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
            ScoreHighPlace,
            ScoreMidPlace,
            ScoreLow,
            Floor,
            FloorAlt,
            FloorAltCube,
            Substation,
            Idle,
            IdleShootPosition
        };

        public static final double STAGE_1_OFFSET = 203;
        public static final double STAGE_2_OFFSET = 270;
        public static final double STAGE_3_OFFSET = 210;

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
