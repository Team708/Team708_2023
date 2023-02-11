// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerIntake extends SubsystemBase {

  private CANSparkMax m_intakeMotor;
  
  /** Creates a new Intake. */
  public RollerIntake() {
    m_intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeOn(){
    m_intakeMotor.set(.3);
  }
  
  public void intakeOff(){
    m_intakeMotor.set(0);
  }
}
