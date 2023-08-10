package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


// THIS IS MY BS CODE FOR VISUALIZING THE ROBOT
// IF U DONT LIKE IT THEN DONT USE IT
public class Visualizer {
    private double wireMass = Units.lbsToKilograms(2.84);

    private double joint1Mass = Units.lbsToKilograms(3.54) + wireMass/2;
    private double joint1Length = Units.inchesToMeters(25);
    private double joint1CgRadius = Units.inchesToMeters(12.5);
    private double joint1Moi = SingleJointedArmSim.estimateMOI(joint1Length, joint1Mass);
    private double joint1Gearing = 100;
  
    private double joint2Mass = Units.lbsToKilograms(3.08) + wireMass/2;
    private double joint2Length = Units.inchesToMeters(30);
    private double joint2CgRadius = Units.inchesToMeters(15);
    private double joint2Moi = SingleJointedArmSim.estimateMOI(joint2Length, joint2Mass);
    private double joint2Gearing = 100;
  
    private double joint3Mass = Units.lbsToKilograms(7.78);
    private double joint3Length = Units.inchesToMeters(20);
    private double joint3CgRadius = Units.inchesToMeters(10);
    private double joint3Moi = SingleJointedArmSim.estimateMOI(joint3Length, joint3Mass);
    private double joint3Gearing = 20;

    Color8Bit yellow = new Color8Bit(Color.kYellow);
    Color8Bit green = new Color8Bit(Color.kGreen);
    Color8Bit blue = new Color8Bit(Color.kBlue);

    private final Mechanism2d armSetpoints = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = armSetpoints.getRoot("ArmPivot", 30, 35);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 35, -90));
    private final MechanismLigament2d m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
              "Arm",
              Units.metersToInches(joint1Length)/2,
              Units.radiansToDegrees(0),
              10,
              blue));
    private final MechanismLigament2d m_arm2 = 
        m_arm.append(
            new MechanismLigament2d(
              "Arm2",
              Units.metersToInches(joint2Length)/2,
              Units.radiansToDegrees(0),
              10,
              green
              ));
    private final MechanismLigament2d m_arm3 = 
        m_arm2.append(
            new MechanismLigament2d(
              "Arm3",
              Units.metersToInches(joint3Length)/2,
              Units.radiansToDegrees(0),
              50,
              blue
              ));


    private final Mechanism2d ConeVis = new Mechanism2d(60, 60);
    private final MechanismRoot2d ConePivot = ConeVis.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d Cone = 
        ConePivot.append(
            new MechanismLigament2d(
              "Cone1",
              10,
              Units.radiansToDegrees(Math.toRadians(-120)),
              10,
              yellow
              ));
    private final MechanismLigament2d Cone2 = 
        Cone.append(
            new MechanismLigament2d(
              "Cone2",
              10,
              Units.radiansToDegrees(Math.toRadians(-60)),
              10,
              yellow
              ));
    private final MechanismLigament2d Cone3 = 
        Cone2.append(
            new MechanismLigament2d(
              "Cone3",
              10,
              Units.radiansToDegrees(Math.toRadians(-120)),
              10,
              yellow
              ));
    
    private final Mechanism2d cubeVis = new Mechanism2d(60, 60);
    private final MechanismRoot2d cubePivot = cubeVis.getRoot("cube{ivot", 30, 30);
    private final MechanismLigament2d cube = 
        cubePivot.append(
            new MechanismLigament2d(
              "cube1",
              10,
              Units.radiansToDegrees(Math.toRadians(0)),
              10,
              yellow
              ));
    private final MechanismLigament2d cube2 = 
        cube.append(
            new MechanismLigament2d(
              "cube2",
              10,
              Units.radiansToDegrees(Math.toRadians(0)),
              10,
              yellow
              ));
    private final MechanismLigament2d cube3 = 
        cube2.append(
            new MechanismLigament2d(
              "cube3",
              10,
              Units.radiansToDegrees(Math.toRadians(0)),
              10,
              yellow
              ));

        
    public Visualizer() {
        SmartDashboard.putData("Cone Visualizer", ConeVis);
        SmartDashboard.putData("Arm Setpoint Visualizer", armSetpoints);
        SmartDashboard.putData("Cube Visualizer", cubeVis);
    }



}
