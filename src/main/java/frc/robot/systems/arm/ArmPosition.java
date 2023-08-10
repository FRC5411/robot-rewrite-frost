package frc.robot.systems.arm;


import frc.robot.systems.arm.ArmVars.Sets;

public class ArmPosition {
    private double stage1Angle = 0;
    private double stage2Angle = 0;
    private double stage3Angle = 0;

    public ArmPosition (double stage1Angle, double stage2Angle, double stage3Angle) {
        this.stage1Angle = stage1Angle;
        this.stage2Angle = stage2Angle;
        this.stage3Angle = stage3Angle;
    }

    public double getStage1Angle () {
        return stage1Angle;
    }

    public double getStage2Angle () {
        return stage2Angle;
    }

    public double getStage3Angle () {
        return stage3Angle;
    }

    public double getStage1OffsetAngle () {
        return stage1Angle - Sets.stageOneJoint.kArmOffsetDeg;
    }

    public double getStage2OffsetAngle () {
        return stage2Angle - Sets.stageTwoJoint.kArmOffsetDeg;
    }

    public double getStage3OffsetAngle () {
        return stage3Angle - Sets.stageThreeJoint.kArmOffsetDeg;
    }

    public double getXPosition() {
        return Sets.stageOneJoint.kArmLength * Math.cos(Math.toRadians(stage1Angle)) + Sets.stageTwoJoint.kArmLength * Math.cos(Math.toRadians(stage2Angle));
    }

    public double getYPosition() {
        return Sets.stageOneJoint.kArmLength * Math.sin(Math.toRadians(stage1Angle) + Sets.stageTwoJoint.kArmLength * Math.sin(Math.toRadians(stage2Angle)));
    }
}