// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.arm.ArmVars.Constants;
import frc.robot.systems.arm.ArmVars.Sets;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;

public class ArmJoint {
    private CANSparkMax jointMotor;
    private DutyCycleEncoder jointEncoder;
    private ArmFeedforward jointFeedforward;
    private double jointTolerance;
    public ProfiledPIDController jointPID;
    public double jointSetpoint;
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
                jointOffsetDeg = Sets.stageOneJoint.kArmOffsetDeg;
                break;
            
            case 3:
                jointMotor = Sets.stageThreeJoint.kArmMotor;
                jointEncoder = Sets.stageThreeJoint.kArmEncoder;
                jointFeedforward = Sets.stageThreeJoint.kArmFF;
                jointTolerance = Sets.stageThreeJoint.kTolerance;
                jointPID = Sets.stageThreeJoint.kArmPID;
                jointOffsetDeg = Sets.stageOneJoint.kArmOffsetDeg;
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
        return getEncoderValue() - jointOffsetDeg;
    }
    
    public double getArmErrorVal(){
        return getEncoderValue() - jointSetpoint;
    }
    

    public Rotation2d getRotation() {
        return new Rotation2d(Math.toRadians(getEncoderValue()));
    }

    public Rotation2d getOffsetRotation() {
        return new Rotation2d(Math.toRadians(getOffsetEncValue()));
    }

    public void runPIDVolts() {
        setJointVolts(
            jointPID.calculate(getOffsetEncValue(), jointSetpoint) 
            +
            jointFeedforward.calculate(
                Math.toRadians(jointPID.getSetpoint().position), 
                Math.toRadians(jointPID.getSetpoint().velocity)));
    }

    public void holdJoint() {
        setJointVolts(
            jointFeedforward.calculate(
                getOffsetEncValue(), 0));
    }

    public void executeControl(BooleanSupplier angleDeadZone, BooleanSupplier goingTuck) {
        runPIDVolts();
    }

    public void resetProfiles(){
        jointPID.reset(getRotation().getDegrees());
    }

    public double getError() {
        return jointPID.getPositionError();
    }

    public boolean inDeadzone(double deadzone) {
        boolean inDeadzone = getError() <= deadzone;
        return inDeadzone;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Volts", jointMotor.get());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Encoder", getEncoderValue());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Error", getError());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Setpoint", jointSetpoint);
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Offset Encoder", getOffsetEncValue());
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Offset Error", getOffsetEncValue() - jointSetpoint);
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Offset Setpoint", jointSetpoint - jointOffsetDeg);
        SmartDashboard.putNumber("Arms/" + Double.toString(jointNum) + "/Voltage", jointMotor.get() * 12);
    }
}