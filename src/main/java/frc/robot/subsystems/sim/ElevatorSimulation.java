// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class ElevatorSimulation {

  private static final double kCarriageMass = 4.0; // kg

  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  private CANSparkMax motorA, motorB;
  private RelativeEncoder encoderA, encoderB;
  private double m_elevatorA, m_elevatorB;

  private final Field2d m_elevatorTrajectorySim;
  Trajectory t = null;

  Elevator e;

  // ElevatorSimulations help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorXSim = new ElevatorSim(
      m_elevatorGearbox,
      ElevatorConstants.kElevatorGearing,
      kCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kMinElevatorReach,
      ElevatorConstants.kMaxElevatorReach,
      false,
      VecBuilder.fill(0.0001));
  private final ElevatorSim m_elevatorZSim = new ElevatorSim(
      m_elevatorGearbox,
      ElevatorConstants.kElevatorGearing,
      kCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kMinElevatorHeight,
      ElevatorConstants.kMaxElevatorHeight,
      false,
      VecBuilder.fill(0.0001));// TODO Make gravety true

  // Create a Mechanism2d visualization of the elevator
  private final Double screenWidth = SimConstants.kChargeStationWidth * 0.5 + SimConstants.kCommunitWidth
      + SimConstants.kGridDepth; // m
  private final double screenHeight = 2.00; // m
  private final double m_elevatorOriginX = (screenWidth - SimConstants.kGridDepth
      - ElevatorConstants.kRobotBumperThickness);
  private final double m_elevatorOriginY = 0.0;
  private final double m_elevatorPositionX = m_elevatorOriginX - ElevatorConstants.kElevatorSetbackFromOrigin;

  private final Mechanism2d m_mech2d = new Mechanism2d(screenWidth, screenHeight);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", m_elevatorPositionX, ElevatorConstants.kElevatorHeightFromOrigin);

  private final MechanismLigament2d m_elevatorLiftMech2d = m_mech2dRoot.append(new MechanismLigament2d("ElevatorLift",
      1,ElevatorConstants.kElevatorAngle, 5, new Color8Bit(Color.kRed)));

  private final MechanismLigament2d m_elevatorArmMech2d = m_elevatorLiftMech2d
      .append(new MechanismLigament2d("ElevatorArm",
          1, -ElevatorConstants.kElevatorAngle, 5, new Color8Bit(Color.kPurple)));

//   private final MechanismLigament2d m_elevatorArm2Mech2d = m_elevatorArmMech2d
//       .append(new MechanismLigament2d("ElevatorArm2",
//           0.10, -90, 5, new Color8Bit(Color.kPurple)));

//   private final MechanismLigament2d m_endEffectorMech2d = m_elevatorArm2Mech2d
//       .append(new MechanismLigament2d("End Effector",
//           ElevatorConstants.kEndEffectorLength, 90, 5, new Color8Bit(Color.kPurple)));

  public ElevatorSimulation(Elevator e, CANSparkMax motorA, CANSparkMax motorB) {
    this.e = e;
    this.motorA = motorA;
    this.motorB = motorB;
    this.encoderA = motorA.getEncoder();
    this.encoderB = motorB.getEncoder();
    REVPhysicsSim.getInstance().addSparkMax(motorA, m_elevatorGearbox);
    REVPhysicsSim.getInstance().addSparkMax(motorB, m_elevatorGearbox);

    m_elevatorTrajectorySim = new Field2d(); // Screen Width 3.885m and Screen Height 2.000m

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator Sim in the simulator, select Network Tables ->
    // SmartDashboard ->
    // Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SmartDashboard.putData("Elevator Trajectory Sim", m_elevatorTrajectorySim);

    makeLine(ElevatorConstants.kLeftBound, ElevatorConstants.kBumperCoord2, 
        ElevatorConstants.kLeftBound, ElevatorConstants.kUpperBound, "m_leftBound");
    makeLine(ElevatorConstants.kLeftBound, ElevatorConstants.kUpperBound, 
        ElevatorConstants.kRightBound, ElevatorConstants.kUpperBound, "m_upperBound");
    makeLine(ElevatorConstants.kRightBound, ElevatorConstants.kHighConeUpperBound, 
        ElevatorConstants.kRightBound, ElevatorConstants.kUpperBound, "m_rightBound");
    makeLine(ElevatorConstants.kLeftBound, ElevatorConstants.kBumperCoord2, 
        ElevatorConstants.kBumperCoord1, ElevatorConstants.kBumperCoord2, "m_bumperTopBound");
    makeLine(ElevatorConstants.kBumperCoord1, 0, 
        ElevatorConstants.kBumperCoord1, ElevatorConstants.kBumperCoord2, "m_bumperSideBound");
    makeLine(ElevatorConstants.kMiddleBound, 0, 
        ElevatorConstants.kMiddleBound, ElevatorConstants.kCubeMiddleShelf, "m_middleBound");
    
    makeLine(ElevatorConstants.kLowConeLeftBound, ElevatorConstants.kCubeMiddleShelf, 
        ElevatorConstants.kLowConeLeftBound, ElevatorConstants.kLowConeUpperBound, "m_lowConeLeftBound");
    makeLine(ElevatorConstants.kMiddleBound, ElevatorConstants.kCubeMiddleShelf, 
        ElevatorConstants.kLowConeLeftBound, ElevatorConstants.kCubeMiddleShelf, "m_middleBoundLimit");
    makeLine(ElevatorConstants.kLowConeLeftBound, ElevatorConstants.kLowConeUpperBound, 
        ElevatorConstants.kCubeMiddleShelfBack, ElevatorConstants.kLowConeUpperBound, "m_lowConeUpperBound");
    makeLine(ElevatorConstants.kCubeMiddleShelfBack, ElevatorConstants.kLowConeUpperBound, 
        ElevatorConstants.kCubeMiddleShelfBack, ElevatorConstants.kCubeTopShelf, "m_cubeMiddleShelfBack");
    makeLine(ElevatorConstants.kCubeMiddleShelfBack, ElevatorConstants.kCubeTopShelf, 
        ElevatorConstants.kHighConeLeftBound, ElevatorConstants.kCubeTopShelf, "m_cubeTopShelf");
    makeLine(ElevatorConstants.kHighConeLeftBound, ElevatorConstants.kCubeTopShelf, 
        ElevatorConstants.kHighConeLeftBound, ElevatorConstants.kHighConeUpperBound, "m_highConeLeftBound");
    makeLine(ElevatorConstants.kHighConeLeftBound, ElevatorConstants.kHighConeUpperBound, 
        ElevatorConstants.kRightBound, ElevatorConstants.kHighConeUpperBound, "m_highConeUpperBound");

    makePoint(ElevatorConstants.kGroundPickupPose, "GroundPickupPose");
    makePoint(ElevatorConstants.kGroundSafePose, "GroundSafePose");  
    makePoint(ElevatorConstants.kStartPose, "StartPose");  
    makePoint(ElevatorConstants.kLowConePose, "LowConePose");
    makePoint(ElevatorConstants.kLowCubePose, "LowCubePose");
    makePoint(ElevatorConstants.kLowSafePose, "LowSafePose");
    makePoint(ElevatorConstants.kMidSafePose, "MidSafePose");
    makePoint(ElevatorConstants.kHighConePose, "HighConePose");
    makePoint(ElevatorConstants.kHighCubePose, "HighCubePose");
    makePoint(ElevatorConstants.kHighSafePose, "HighSafePose");

    encoderA.setPosition(0.5);
    encoderB.setPosition(0.5);
  }

  // Boundary Lines
  // private final MechanismRoot2d m_leftBoundStart =
  // m_mech2d.getRoot("m_leftBoundStart", m_OriginX-ElevatorConstants.kLeftBound,
  // 0);
  // private final MechanismLigament2d m_leftBoundEnd =
  // m_leftBoundStart.append( new MechanismLigament2d("ElevatorLift",
  // ElevatorConstants.kUpperBound, 90,5,new Color8Bit(Color.kRed)));

  public void update(Double m_setposX, Double m_setposZ) {
    REVPhysicsSim.getInstance().run();

    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorXSim.setInput(motorB.getAppliedOutput() + (motorA.getAppliedOutput() * ElevatorConstants.kElevatorCosAngle));
    m_elevatorZSim.setInput(motorA.getAppliedOutput() *  ElevatorConstants.kElevatorSinAngle);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorXSim.update(GlobalConstants.kLoopTime);
    m_elevatorZSim.update(GlobalConstants.kLoopTime);

    m_elevatorA = m_elevatorZSim.getPositionMeters() / ElevatorConstants.kElevatorSinAngle;
    m_elevatorB = m_elevatorXSim.getPositionMeters() - (m_elevatorZSim.getPositionMeters() / Math.tan(Math.toRadians(ElevatorConstants.kElevatorAngle)));
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    encoderA.setPosition(m_elevatorA);
    encoderB.setPosition(m_elevatorB);

    // Update elevator visualization with simulated position
    m_elevatorLiftMech2d.setLength(m_elevatorA);
    m_elevatorArmMech2d.setLength(m_elevatorB + ElevatorConstants.kElevatorSetbackFromOrigin);

    Pose2d elevatorPose = new Pose2d(
        new Translation2d(m_elevatorOriginX + m_setposX, m_setposZ), new Rotation2d());

    m_elevatorTrajectorySim.setRobotPose(elevatorPose);
    if (t != null)
      m_elevatorTrajectorySim.getRobotObject().setTrajectory(
          t.relativeTo(new Pose2d(-m_elevatorOriginX,
              Elevator.J.getPose().getY(),
              new Rotation2d(0))));
  }

  public void drawTrajectory(Trajectory t) {
    this.t = t;
  }

  private void makeLine(double x1, double y1, double x2, double y2, String name) {
    double xDist = x2 - x1;
    double yDist = y2 - y1;
    double hypotenuse = Math.hypot(xDist, yDist);
    double lineAngle = Math.toDegrees(Math.atan2(yDist, xDist));
    MechanismRoot2d m_lineStart = m_mech2d.getRoot(name + "Start", m_elevatorOriginX + x1, m_elevatorOriginY + ElevatorConstants.kElevatorHeightFromOrigin + y1);
    MechanismLigament2d m_lineEnd = m_lineStart.append(new MechanismLigament2d(name + "End",
        hypotenuse, lineAngle, 3, new Color8Bit(Color.kYellowGreen)));
  }

  private void makePoint(double x, double y, String name){
    MechanismRoot2d m_lineStart = m_mech2d.getRoot(name + "Start", m_elevatorOriginX + x, m_elevatorOriginY + ElevatorConstants.kElevatorHeightFromOrigin + y);
    MechanismLigament2d m_lineEnd = m_lineStart.append( new MechanismLigament2d(name +"End", 0.015, 0, 5, new Color8Bit(Color.kRed)));
  }

  private void makePoint(Translation2d point, String name){
    makePoint(point.getX(), point.getY(), name);
  }
}
