package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.systems.arm.ArmPosition;
import frc.robot.systems.intake.IntakeVars.GamePieces;


// THIS IS MY BS CODE FOR VISUALIZING THE ROBOT
// IF U DONT LIKE IT THEN DONT USE IT
public class Visualizer {
    private double wireMass;

    private double joint1Mass;
    private double joint1Length;
    private double joint1CgRadius;
    private double joint1Moi;
    private double joint1Gearing;
  
    private double joint2Mass;
    private double joint2Length;
    private double joint2CgRadius;
    private double joint2Moi;
    private double joint2Gearing;
  
    private double joint3Mass;
    private double joint3Length;
    private double joint3CgRadius;
    private double joint3Moi;
    private double joint3Gearing;

    Color8Bit yellow = new Color8Bit(Color.kYellow);
    Color8Bit green = new Color8Bit(Color.kGreen);
    Color8Bit blue = new Color8Bit(Color.kBlue);
    Color8Bit purple = new Color8Bit(Color.kPurple);

    private Mechanism2d armSetpoints;
    private MechanismRoot2d m_armPivot;
    private MechanismLigament2d m_armTower;
    private MechanismLigament2d m_arm;
    private MechanismLigament2d m_arm2;
    private MechanismLigament2d m_arm3;
    
    private Mechanism2d cubeVis;
    private MechanismRoot2d cubePivot;
    private MechanismLigament2d cube;
    private MechanismLigament2d cube2;
    private MechanismLigament2d cube3;
    private MechanismLigament2d cube4;

        
    public Visualizer() {
        wireMass = Units.lbsToKilograms(2.84);

        joint1Mass = Units.lbsToKilograms(3.54) + wireMass/2;
        joint1Length = Units.inchesToMeters(25);
        joint1CgRadius = Units.inchesToMeters(12.5);
        joint1Moi = SingleJointedArmSim.estimateMOI(joint1Length, joint1Mass);
        joint1Gearing = 100;
      
        joint2Mass = Units.lbsToKilograms(3.08) + wireMass/2;
        joint2Length = Units.inchesToMeters(30);
        joint2CgRadius = Units.inchesToMeters(15);
        joint2Moi = SingleJointedArmSim.estimateMOI(joint2Length, joint2Mass);
        joint2Gearing = 100;
      
        joint3Mass = Units.lbsToKilograms(7.78);
        joint3Length = Units.inchesToMeters(20);
        joint3CgRadius = Units.inchesToMeters(10);
        joint3Moi = SingleJointedArmSim.estimateMOI(joint3Length, joint3Mass);
        joint3Gearing = 20;
    
        Color8Bit yellow = new Color8Bit(Color.kYellow);
        Color8Bit green = new Color8Bit(Color.kGreen);
        Color8Bit blue = new Color8Bit(Color.kBlue);
        Color8Bit purple = new Color8Bit(Color.kPurple);
    
        armSetpoints = new Mechanism2d(60, 60);
        m_armPivot = armSetpoints.getRoot("ArmPivot", 30, 35);
        m_armTower =
            m_armPivot.append(new MechanismLigament2d("ArmTower", 35, -90));
        m_arm =
            m_armPivot.append(
                new MechanismLigament2d(
                  "Arm",
                  Units.metersToInches(joint1Length)/2,
                  Units.radiansToDegrees(0),
                  10,
                  blue));
        m_arm2 = 
            m_arm.append(
                new MechanismLigament2d(
                  "Arm2",
                  Units.metersToInches(joint2Length)/2,
                  Units.radiansToDegrees(0),
                  10,
                  green
                  ));
        m_arm3 = 
            m_arm2.append(
                new MechanismLigament2d(
                  "Arm3",
                  Units.metersToInches(joint3Length)/2,
                  Units.radiansToDegrees(0),
                  50,
                  blue
                  ));
        
        cubeVis = new Mechanism2d(60, 60);
        cubePivot = cubeVis.getRoot("cubePivot", 30, 30);
        cube = 
            cubePivot.append(
                new MechanismLigament2d(
                  "cube1",
                  10,
                  Units.radiansToDegrees(Math.toRadians(90)),
                  10,
                  purple
                  ));
        cube2 = 
            cube.append(
                new MechanismLigament2d(
                  "cube2",
                  10,
                  Units.radiansToDegrees(Math.toRadians(90)),
                  10,
                  purple
                  ));
        cube3 = 
            cube2.append(
                new MechanismLigament2d(
                  "cube3",
                  10,
                  Units.radiansToDegrees(Math.toRadians(90)),
                  10,
                  purple
                  ));
        cube4 = 
            cube3.append(
                new MechanismLigament2d(
                  "cube4",
                  10,
                  Units.radiansToDegrees(Math.toRadians(90)),
                  10,
                  purple
                  ));
        
        SmartDashboard.putData("GP Visualizer", cubeVis);
        SmartDashboard.putData("Arm Setpoint Visualizer", armSetpoints);
    }

    public void updateGP(GamePieces GP) {
        if(GP == GamePieces.Cone) {
            cube.setColor(yellow);
            cube2.setColor(yellow);
            cube3.setColor(yellow);
            cube4.setColor(yellow);
        }
        else if(GP == GamePieces.Cube) {
            cube.setColor(purple);
            cube2.setColor(purple);
            cube3.setColor(purple);
            cube4.setColor(purple);
        }
        else if(GP == GamePieces.None) {
            cube.setColor(green);
            cube2.setColor(green);
            cube3.setColor(green);
            cube4.setColor(green);
        }
    }

    public void updateArms(ArmPosition position) {
        m_arm.setAngle(new Rotation2d(Math.toRadians(position.getStage1OffsetAngle())));
        m_arm2.setAngle(new Rotation2d(Math.toRadians(position.getStage2OffsetAngle() - m_arm.getAngle())));
        m_arm3.setAngle(new Rotation2d(Math.toRadians(position.getStage3OffsetAngle() - m_arm2.getAngle()) - m_arm.getAngle()));
    }
}
