// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.systems.arm.ArmVars.Sets;
import frc.robot.systems.arm.ArmVars.Objects;

/** Add your docs here. */
public class ArmIO {
    public double[] returnForwardKinematics(double stage1Degrees, double stage2Degrees, double stage3Degrees) {
        double[] output = new double[3];
    
        Rotation2d stage1Rotation = new Rotation2d(Math.toRadians(stage1Degrees));
        Rotation2d stage2Rotation = new Rotation2d(Math.toRadians(stage2Degrees));
        Rotation2d stage3Rotation = new Rotation2d(Math.toRadians(stage3Degrees));
        
        output[0] = stage1Rotation.getCos() * Sets.stageOneJoint.kArmLength + stage2Rotation.getCos() * Sets.stageTwoJoint.kArmLength;
        output[1] = stage1Rotation.getSin() * Sets.stageOneJoint.kArmLength + stage2Rotation.getSin() * Sets.stageTwoJoint.kArmLength;
        output[2] = (360 + stage3Rotation.getDegrees()) % 360;
    
        return output;
    }

    public void telemetry() {
        Objects.jointStageOne.telemetry();
        Objects.jointStageTwo.telemetry();
        Objects.jointStageThree.telemetry();
    }
}
