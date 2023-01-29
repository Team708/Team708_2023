// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.sim.ElevatorSimulation;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private CANSparkMax m_elevatorMotorA,m_elevatorMotorB;
  private RelativeEncoder m_encoderA,m_encoderB;
  private double m_setposX,m_setposZ;
  private PIDController m_pidControllerX,m_pidControllerZ;
  private double outputX,outputZ,outputA,outputB;
  private double m_measureX,m_measureZ;
  
  private ElevatorSimulation m_elevatorSim;

  public Elevator() {

    m_elevatorMotorA = new CANSparkMax(ElevatorConstants.kMotorAID, MotorType.kBrushless);
    m_elevatorMotorB = new CANSparkMax(ElevatorConstants.kMotorBID, MotorType.kBrushless);
    m_elevatorMotorA.setIdleMode(IdleMode.kBrake);
    m_elevatorMotorB.setIdleMode(IdleMode.kBrake);
    m_encoderA = m_elevatorMotorA.getEncoder();
    m_encoderB = m_elevatorMotorB.getEncoder();
    m_encoderA.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderConversionFactor);
    m_encoderB.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderConversionFactor);
    m_pidControllerX = new PIDController(ElevatorConstants.kPID[0], ElevatorConstants.kPID[1], ElevatorConstants.kPID[2]);
    m_pidControllerZ = new PIDController(ElevatorConstants.kPID[0], ElevatorConstants.kPID[1], ElevatorConstants.kPID[2]);
    
  }

  @Override
  public void periodic() {
    //gets the current position of elevator
    m_measureX = getX();
    m_measureZ = getZ();
    SmartDashboard.putNumber("measureX", m_measureX);
    SmartDashboard.putNumber("measureZ", m_measureZ);
    SmartDashboard.putNumber("measureA", m_encoderA.getPosition());
    SmartDashboard.putNumber("measureB", m_encoderB.getPosition());
    
    //outputX calculates the error between getX and setposX
    outputX = m_pidControllerX.calculate(m_measureX, m_setposX);
    //outputZ calculates the error between getZ and setposZ
    outputZ = m_pidControllerZ.calculate(m_measureZ, m_setposZ);
    //convert cartesian setpoints to motor positions
    outputA = getA(outputX, outputZ);
    outputB = getB(outputX, outputZ);
    //set motion positions
    m_elevatorMotorA.set(outputA);
    m_elevatorMotorB.set(outputB);
  }

  public void simulationInit(){
   //Setup the elevator simulation
   m_elevatorSim = new ElevatorSimulation(m_elevatorMotorA, m_elevatorMotorB);
  }

  @Override
  public void simulationPeriodic(){
    //Update elevator simulation
    m_elevatorSim.update(m_setposX, m_setposZ);
  }

  /**
  * @param A double position of motor A
  * @param B double position of motor B
  * @return horizontal position
  */ 
  public double getX() {
    return 0.5*(m_encoderA.getPosition() + m_encoderB.getPosition()); 
  }
  /**
  * @param A double position of motor A
  * @param B double position of motor B
  * @return vertical position
  */ 
  public double getZ() {
    return 0.5*(m_encoderA.getPosition() - m_encoderB.getPosition());
  }
  /**
   * @param X horizontal distance
   * @param Z vertical distance
   * @return position of motor A
   */
  public double getA(double X, double Z) {
    return X+Z;
  }
  /**
   * @param X horizontal distance
   * @param Z vertical distance
   * @return position of motor B
   */
  public double getB(double X, double Z) {
    return X-Z;
  }

  public Translation2d getPose() {
    return new Translation2d(getX(),getZ());
  }

  public void setX(double X){
    m_setposX = X;
  }

  public void setZ(double Z){
    m_setposZ = Z;
  }

  public void setPose(Translation2d elevatorPose) {
    m_setposX = elevatorPose.getX();
    m_setposZ = elevatorPose.getY();
  } 

  public void sendToDashboard(){
    SmartDashboard.putNumber("elevatorPoseX", m_setposX);
    SmartDashboard.putNumber("elevatorPoseZ", m_setposZ);
  }
}
