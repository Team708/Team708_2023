// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Utilities.DecisionTree.Branch;
import frc.robot.Utilities.DecisionTree.BranchExceptionError;
import frc.robot.Utilities.DecisionTree.Node;
import frc.robot.Utilities.DecisionTree.Tree;
import frc.robot.subsystems.sim.ElevatorSimulation;

public class Elevator extends SubsystemBase {
  /* Creates a new Elevator. */
  private CANSparkMax m_elevatorMotorA, m_elevatorMotorB;
  private RelativeEncoder m_encoderA, m_encoderB;
  private double m_setposX, m_setposZ;
  private ProfiledPIDController m_pidControllerX, m_pidControllerZ;
  private double outputX, outputZ, outputA, outputB;
  private double m_measureX, m_measureZ;

    //collision check
  private boolean isColliding = false;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
  private final SlewRateLimiter m_slewZ = new SlewRateLimiter(12.0);

  private ElevatorSimulation m_elevatorSim = null;

  private Node elevatorCurrentNode;

  public static final Node A = new Node(ElevatorConstants.kGroundPickupPose, "GROUND_PICKUP");
  public static final Node B = new Node(ElevatorConstants.kGroundSafePose, "GROUND_SAFE");
  public static final Node C = new Node(ElevatorConstants.kHighConePose, "HIGH_CONE");
  public static final Node D = new Node(ElevatorConstants.kHighCubePose, "HIGH_CUBE");
  public static final Node E = new Node(ElevatorConstants.kHighSafePose, "HIGH_SAFE");
  public static final Node F = new Node(ElevatorConstants.kLowConePose, "LOW_CONE");
  public static final Node G = new Node(ElevatorConstants.kLowCubePose, "LOW_CUBE");
  public static final Node H = new Node(ElevatorConstants.kLowSafePose, "LOW_SAFE");
  public static final Node I = new Node(ElevatorConstants.kMidSafePose, "MID_SAFE");
  public static final Node J = new Node(ElevatorConstants.kStartPose, "START");
  private HashMap<String, Node> map;

  private Branch AB = new Branch(A, B); //Ground -> GS
  private Branch BJ = new Branch(B, J); //GS -> Start
  private Branch BH = new Branch(B, H); //GS -> LS
  private Branch HG = new Branch(H, G); //LS -> LCUBE
  private Branch HI = new Branch(H, I); //LS -> MS
  private Branch IF = new Branch(I, F); //MS -> LCONE
  private Branch FD = new Branch(F, D); //LCONE -> HCUBE
  private Branch IE = new Branch(I, E); //MS -> HS
  private Branch EC = new Branch(E, C); //HS -> HCONE

  private Tree nodeTree;

  public Elevator() {

    //The elevator uses NEOs for both forward and upward motion.
    m_elevatorMotorA = new CANSparkMax(ElevatorConstants.kMotorAID, MotorType.kBrushless);
    m_elevatorMotorB = new CANSparkMax(ElevatorConstants.kMotorBID, MotorType.kBrushless);
    m_elevatorMotorA.setIdleMode(IdleMode.kBrake);
    m_elevatorMotorB.setIdleMode(IdleMode.kBrake);

    //Creates the encoders for the tracking of the elevator position in meters. 
    //X and Z positions require knowing both positions of motors A and B 
    m_encoderA = m_elevatorMotorA.getEncoder();
    m_encoderB = m_elevatorMotorB.getEncoder();
    m_encoderA.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderConversionFactorA);
    m_encoderB.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderConversionFactorB);
    
    //Create a PID controller for both X and Z direction. Since X moves horizontally and Z moves vertically
    //each axis will require unique PID gains.
    m_pidControllerX = new ProfiledPIDController(ElevatorConstants.kPID_X[0], ElevatorConstants.kPID_X[1], ElevatorConstants.kPID_X[2], new Constraints(30.0,  10));
    m_pidControllerZ = new ProfiledPIDController(ElevatorConstants.kPID_Z[0], ElevatorConstants.kPID_Z[1], ElevatorConstants.kPID_Z[2],new Constraints(30.0,  10));

    //Generate a list of nodes as waypoints for the elevator
    map = new HashMap<String, Node>();
    map.put(A.getIdentifier(), A);
    map.put(B.getIdentifier(), B);
    map.put(C.getIdentifier(), C);
    map.put(D.getIdentifier(), D);
    map.put(E.getIdentifier(), E);
    map.put(F.getIdentifier(), F);
    map.put(G.getIdentifier(), G);
    map.put(H.getIdentifier(), H);
    map.put(I.getIdentifier(), I);
    map.put(J.getIdentifier(), J);

    nodeTree = new Tree(map);
    for (Branch branch : getElevatorBranches()) {
        try{
            nodeTree.addBranch(branch);
        }catch(BranchExceptionError e){
            e.printStackTrace();
        }
    }

    this.elevatorCurrentNode = J;
  }

  @Override
  public void periodic() {
    //gets the current position of elevator
    m_measureX = getX();
    m_measureZ = getZ();
    checkBoundary(m_setposX, m_measureZ);
    //outputX calculates the error between getX and setposX
    outputX = m_pidControllerX.calculate(m_measureX, m_setposX);

    //outputZ calculates the error between getZ and setposZ
    outputZ = m_pidControllerZ.calculate(m_measureZ, m_setposZ);
        
    //convert cartesian setpoints to motor positions
    outputA = getA(outputZ);
    outputB = getB(outputX, outputZ);

    //set motion positions
    m_elevatorMotorA.setVoltage(outputA);
    m_elevatorMotorB.setVoltage(outputB);

  }

  public void simulationInit(){
   //Setup the elevator simulation
   m_elevatorSim = new ElevatorSimulation(this, m_elevatorMotorA, m_elevatorMotorB);
  }

  @Override
  public void simulationPeriodic(){
    //Update elevator simulation
    m_elevatorSim.update(m_setposX, m_setposZ);
  }

  public ElevatorSimulation getSim(){
    return m_elevatorSim;
  }

  /**
  * @return Horizontal position
  */ 
  public double getX() {
    return m_encoderB.getPosition() + (m_encoderA.getPosition() * ElevatorConstants.kElevatorCosAngle); 
  }

  /**
  * @return Vertical position
  */ 
  public double getZ() {
    return m_encoderA.getPosition() *  ElevatorConstants.kElevatorSinAngle;
  }

  /**
  * @return Horizontal velocity
  */ 
  public double getXVel() {
    return m_encoderB.getVelocity() + (m_encoderA.getVelocity() * ElevatorConstants.kElevatorCosAngle);
  }

  /**
  * @return Vertical velocity
  */ 
  public double getZVel() {
    return m_encoderA.getVelocity() * ElevatorConstants.kElevatorSinAngle;
  }

  /**
   * @param X horizontal distance
   * @param Z vertical distance
   * @return position of motor A
   */
  public double getA(double Z) {
    return Z / ElevatorConstants.kElevatorSinAngle;
  }

  /**
   * @param X horizontal distance
   * @param Z vertical distance
   * @return position of motor B
   */
  public double getB(double X, double Z) {
    return X - Z / ElevatorConstants.kElevatorTanAngle;
  }

  /**
   * Sets the desired X position 
   * @param X position
   */
  public void setX(double X){
    m_setposX = X;
  }

  /**
   * Sets the desired Z position 
   * @param Z position
   */
  public void setZ(double Z){
    m_setposZ = Z;
  }

  /**
   * Position of the elevator. 
   * X-axis relative to leading edge of robot frame,
   * Z-axis relative to ground
   * @return Translation2d position of the elevator.
   */
  public Translation2d getPose() {
    return new Translation2d(getX(),getZ());
  }

  /**
   * Position of the elevator. 
   * X-axis relative to leading edge of robot frame,
   * Z-axis relative to ground
   * @return Pose2d position of the elevator.
   */
  public Pose2d getPose2d(){
    return new Pose2d(new Translation2d(getX(), getZ()), new Rotation2d(0));
  }

  // public void setPose(Pose2d pose){
  //   m_setposX = pose.getX();
  //   m_setposZ = pose.getY();
  // }

  public void setPose(Translation2d elevatorPose) {
    m_setposX = elevatorPose.getX();
    m_setposZ = elevatorPose.getY();
  } 

  public Node getElevatorNode(){
    return elevatorCurrentNode;
  }

  public Node getClosestNode(){
    List<Translation2d> temp = new ArrayList<Translation2d>();
    nodeTree.getNodes().stream().forEach(i -> temp.add(i.getPosition()));
    Translation2d nearest = getPose().nearest(temp);
    for(Node n : nodeTree.getNodes()){
      if(n.getPosition().getX() == nearest.getX() && n.getPosition().getY() == nearest.getY()){
        return n;
      }
    }
    return null;
  }

  public void setElevatorNode(Node n){
    this.elevatorCurrentNode = n;
  }

  public HashMap<String, Node> getMap(){
    return map;
  }

  /**
   * @return List of branches between nodes
   */
  public Branch[] getElevatorBranches(){
    return new Branch[]{AB, BJ, BH, HG, HI, IF, FD, IE, EC};
  }

  public Tree getElevatorTree(){
    return nodeTree;
  }


   /**
   * Method to drive the elevator using joystick inputs.
   *
   * @param xSpeed        Speed of the elevator in the x direction (forward).
   * @param zSpeed        Speed of the elevator in the z direction (up).
   */
  public void commandedVelocity(double xSpeed, double zSpeed) {
    m_setposX = m_measureX + (xSpeed * GlobalConstants.kLoopTime);
    m_setposZ = m_measureZ +(zSpeed *  GlobalConstants.kLoopTime);
  }

  public double elevatorDiagX(double Z){
    return ((Z - ElevatorConstants.diagy1) / ElevatorConstants.kElevatorTanAngle) + ElevatorConstants.kLeftBound;
  }

  public double elevatorDiagZ(double X){
    return ElevatorConstants.kElevatorTanAngle * (X - ElevatorConstants.kLeftBound) + ElevatorConstants.diagy1;
  }

  public void drawSimTrajectory(Trajectory t){
    m_elevatorSim.drawTrajectory(t);
  }

  private void checkBoundary(double X, double Z){
    //outer bounds
    if (m_setposX < ElevatorConstants.kLeftBound && outputX < 0) {
      m_setposX = ElevatorConstants.kLeftBound;
      isColliding = true;
    } else if(m_setposX > ElevatorConstants.kRightBound && outputX > 0){
      m_setposX = ElevatorConstants.kRightBound;
      isColliding = true;
    }else if(m_setposZ < ElevatorConstants.kLowerBound && outputZ < 0){
      m_setposZ = ElevatorConstants.kLowerBound;
      isColliding = true;
    }else if(m_setposZ > ElevatorConstants.kUpperBound && outputZ > 0){
      m_setposZ = ElevatorConstants.kUpperBound;
      isColliding = true;
    }

    //bumper bounds
    if(m_setposX < ElevatorConstants.kBumperCoord1 && 
        m_measureZ < ElevatorConstants.kBumperCoord2 && outputX < 0){
      m_setposX = ElevatorConstants.kBumperCoord1;
      isColliding = true;
    }
    if(m_measureX < ElevatorConstants.kBumperCoord1 && 
        m_setposZ < ElevatorConstants.kBumperCoord2 && outputZ < 0){
      m_setposZ = ElevatorConstants.kBumperCoord2;
      isColliding = true;
    }

    //grid bounds
    if(m_measureZ < ElevatorConstants.kCubeMiddleShelf && 
        m_setposX > ElevatorConstants.kMiddleBound){
      m_setposX = ElevatorConstants.kMiddleBound;
      isColliding = true;
     }
    if(m_setposZ < ElevatorConstants.kCubeMiddleShelf && 
    m_measureX > ElevatorConstants.kMiddleBound){
      m_setposZ = ElevatorConstants.kCubeMiddleShelf;
      isColliding = true;
    }
    if(m_measureZ > ElevatorConstants.kCubeMiddleShelf && m_measureZ < ElevatorConstants.kLowConeUpperBound &&
        m_setposX > ElevatorConstants.kLowConeLeftBound){
      m_setposX = ElevatorConstants.kLowConeLeftBound;
      isColliding = true;
    }
    if(m_measureX > ElevatorConstants.kLowConeLeftBound && m_measureX < ElevatorConstants.kCubeMiddleShelfBack
     && ElevatorConstants.kLowConeUpperBound > m_setposZ) {
      m_setposZ = ElevatorConstants.kLowConeUpperBound;
      isColliding = true;
     }
     if(m_measureZ > ElevatorConstants.kLowConeUpperBound && m_measureZ < ElevatorConstants.kCubeTopShelf
     && ElevatorConstants.kCubeMiddleShelfBack < m_setposX) {
      m_setposX = ElevatorConstants.kCubeMiddleShelfBack;
      isColliding = true;
     }
     if(m_measureX > ElevatorConstants.kCubeMiddleShelfBack && m_measureX < ElevatorConstants.kHighConeLeftBound
     && ElevatorConstants.kCubeTopShelf > m_setposZ) {
      m_setposZ = ElevatorConstants.kCubeTopShelf;
      isColliding = true;
     }
     if(m_measureZ < ElevatorConstants.kHighConeUpperBound && m_measureZ > ElevatorConstants.kCubeMiddleShelfBack
      && m_setposX > ElevatorConstants.kHighConeLeftBound) {
      m_setposX = ElevatorConstants.kHighConeLeftBound;
      isColliding = true;
     }
     if(m_measureX < ElevatorConstants.kRightBound + .1 && m_measureX > ElevatorConstants.kHighConeLeftBound
      && m_setposZ < ElevatorConstants.kHighConeUpperBound) {
      m_setposZ = ElevatorConstants.kHighConeUpperBound;
      isColliding = true;
     }

    //  //diagonal bound
    double xLimit = elevatorDiagX(m_measureZ);
    double zLimit = elevatorDiagZ(m_measureX);

     if(m_setposZ > zLimit) {
     m_setposZ = zLimit;
     isColliding = true;
    }
    if(m_setposX < xLimit) {
      m_setposX = xLimit;
      isColliding = true;
     }
    else{
      isColliding = false;
    }

  }

  public void sendToDashboard(){
    SmartDashboard.putNumber("elevatorPoseX", m_setposX);
    SmartDashboard.putNumber("elevatorPoseZ", m_setposZ);
    SmartDashboard.putNumber("measureX", m_measureX);
    SmartDashboard.putNumber("measureZ", m_measureZ);
    SmartDashboard.putNumber("measureA", m_encoderA.getPosition());
    SmartDashboard.putNumber("measureB", m_encoderB.getPosition());
    SmartDashboard.putBoolean("Collision Detected", isColliding);
  }
}