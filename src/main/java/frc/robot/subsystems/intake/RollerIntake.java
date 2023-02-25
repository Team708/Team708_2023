// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class RollerIntake extends SubsystemBase {

  private CANSparkMax m_intakeMotor;
  private RelativeEncoder m_intakeEncoder;
  boolean isReversed = false;

  DigitalInput m_dIOSensor;
  
  /** Creates a new Intake. */
  public RollerIntake(DigitalInput m_dIOSensor) {
    m_intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);

    m_intakeEncoder = m_intakeMotor.getEncoder();

    this.m_dIOSensor = m_dIOSensor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeOn(){
    m_intakeMotor.set(IntakeConstants.kRollerIntakeSpeed);
    isReversed = false;
  }
  
  public void intakeOff(){
    m_intakeMotor.set(0);
  }

  public void intakeReverse(){
    m_intakeMotor.set(-IntakeConstants.kRollerIntakeSpeed); //ESTABLISH SLOWER CONSTANT
    isReversed = true;
  }

  public boolean getReversed(){
    return isReversed;
  }

  public void setReversed(boolean isReversed){
    this.isReversed = isReversed;
  }

  public boolean sensorDetected(){
    return !m_dIOSensor.get();
  }

  public void sendToDashboard(){
    SmartDashboard.putNumber("Intake Count", m_intakeEncoder.getPosition());
    SmartDashboard.putBoolean("Sensor", sensorDetected());
  }

}
