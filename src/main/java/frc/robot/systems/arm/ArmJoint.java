// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.arm.ArmVars.Constants;
import frc.robot.systems.arm.ArmVars.Sets;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class ArmJoint {
    private CANSparkMax jointMotor;
    private DutyCycleEncoder jointEncoder;
    private ArmFeedforward jointFeedforward;
    private double jointTolerance;
    public ProfiledPIDController jointPID;
    public double jointOffsetDeg;
    public int jointNum;

    public ArmJoint(int jointNum) {
        this.jointNum = jointNum;
        initVars(jointNum); // Fetches variables from constants and sets it to this class' variables
        configureVars(); // Configures PID, FF, Motors, etc.
    }

    private void initVars(int jointNum) {
        switch(jointNum) {
            case 1:
                jointMotor = Sets.stageOneJoint.kArmMotor;
                jointEncoder = Sets.stageOneJoint.kArmEncoder;
                jointFeedforward = Sets.stageOneJoint.kArmFF;
                jointTolerance = Sets.stageOneJoint.kTolerance;
                jointPID = Sets.stageOneJoint.kArmPID;
                jointOffsetDeg = Sets.stageOneJoint.kArmOffsetDeg;
                break;
            case 2:
                jointMotor = Sets.stageTwoJoint.kArmMotor;
                jointEncoder = Sets.stageTwoJoint.kArmEncoder;
                jointFeedforward = Sets.stageTwoJoint.kArmFF;
                jointTolerance = Sets.stageTwoJoint.kTolerance;
                jointPID = Sets.stageTwoJoint.kArmPID;
                jointOffsetDeg = Sets.stageTwoJoint.kArmOffsetDeg;
                break;
            
            case 3:
                jointMotor = Sets.stageThreeJoint.kArmMotor;
                jointEncoder = Sets.stageThreeJoint.kArmEncoder;
                jointFeedforward = Sets.stageThreeJoint.kArmFF;
                jointTolerance = Sets.stageThreeJoint.kTolerance;
                jointPID = Sets.stageThreeJoint.kArmPID;
                jointOffsetDeg = Sets.stageThreeJoint.kArmOffsetDeg;
                break;

            default:
                System.out.println("!!! SWITCH CASE ERROR !!!");
        }
    }

    private void configureVars(){
        jointPID.enableContinuousInput(0, 360);
        jointPID.setTolerance(jointTolerance);
        jointPID.setIntegratorRange(Constants.kJointIntegratorMin, Constants.kJointIntegratorMax);
    }

    public void setJointStop(){
        jointMotor.set(0);
    }

    public void voltJointStop(){
        jointMotor.setVoltage(0);
    }
    
    public void setJointVolts(double voltage){
        jointMotor.setVoltage(voltage);
    }
    
    public double getEncoderValue(){
        return jointEncoder.getAbsolutePosition() * 360;
    }

    public double getOffsetEncValue(){
        double val = getEncoderValue() - jointOffsetDeg;
        if(val < 0) {
            val = 360 + val;
        }
        return val;
    }
    

    public Rotation2d getRotation() {
        return new Rotation2d(Math.toRadians(getEncoderValue()));
    }

    public Rotation2d getOffsetRotation() {
        return new Rotation2d(Math.toRadians(getOffsetEncValue()));
    }

    public void runPIDVolts(double setpoint, double invert) {
        double outPut = 
            (12 * jointPID.calculate(
                getEncoderValue() - jointOffsetDeg, setpoint) 
            +
            jointFeedforward.calculate(
                Math.toRadians(jointPID.getSetpoint().position), 
                Math.toRadians(jointPID.getSetpoint().velocity))) * Math.signum(invert);

        double clampedOutPut = MathUtil.clamp(outPut, -12, 12);

        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/outPut", clampedOutPut);

        setJointVolts(MathUtil.clamp(clampedOutPut, -12, 12));
    }

    public void holdJoint() {
        setJointVolts(
            jointFeedforward.calculate(
                getOffsetEncValue(), 0));
    }

    public void executeControl(DoubleSupplier setPointSupplier, double invert) {
        runPIDVolts(setPointSupplier.getAsDouble(), invert);
    }

    public void resetProfiles(){
        jointPID.reset(getOffsetRotation().getDegrees());
    }

    public double getError() {
        return jointPID.getPositionError();
    }

    public boolean inDeadzone(double deadzone) {
        boolean inDeadzone = getError() <= deadzone;
        return inDeadzone;
    }

    public ProfiledPIDController getPID() {
        return jointPID;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Voltage", jointMotor.get() * 12);
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/PercentOuptut", jointMotor.get());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/OffsetDegrees", getOffsetEncValue());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/EncoderDegrees", getEncoderValue());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Error", getError());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Goal", jointPID.getGoal().position);
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Setpoint", jointPID.getSetpoint().position);
    }
}