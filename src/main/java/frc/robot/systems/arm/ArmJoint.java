// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class ArmJoint {
    private CANSparkMax armMotor;
    private DutyCycleEncoder armEncoder;
    private ProfiledPIDController armPID;
    private double armTarget;
    private byte armJointNum;

    private ArmFeedforward armFF;

    private double armKP;
    private double armKI;
    private double armKD;


    public ArmJoint(CANSparkMax motorPar, DutyCycleEncoder encoderPar, ProfiledPIDController pidPar, 
                    double targetPar, byte jointNumPar, ArmFeedforward feedforwardPar) {
        armMotor = motorPar;
        armEncoder = encoderPar;
        armPID = pidPar;
        armTarget = targetPar;
        armJointNum = jointNumPar;
        armFF = feedforwardPar;
    }

    public void initMotorAndEnc(){
        armMotor = new CANSparkMax(armJointNum, null);

        armMotor.restoreFactoryDefaults();
        armMotor.clearFaults();
        armMotor.burnFlash();
        armMotor.setSmartCurrentLimit(40);
        armMotor.setSecondaryCurrentLimit(40);
        armMotor.setIdleMode(IdleMode.kBrake);

    }

    public void initPIDAndFF(){

        // TODO ADD ARM FF TO CONSTANTS
        armFF = new ArmFeedforward(0, 0, 0,0); // 1.37 1.35 1.3
  
        // TODO ADD TRAPEZOIDAL PROFILE CONSTRAINTS TO CONSTANTS
        armPID = new ProfiledPIDController(
            armKP, armKI, armKD, 
            new TrapezoidProfile.Constraints(0, 0)
        );

        armPID.enableContinuousInput(0, 360);


    }
}
