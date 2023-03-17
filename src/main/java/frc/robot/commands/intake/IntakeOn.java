// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;

public class IntakeOn extends CommandBase {

  Intake m_intake;
  Elevator m_elevator;
  int first = 0;

  public IntakeOn(Intake m_intake) {
    this.m_intake = m_intake;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setHasPiece(false);
    m_intake.resetRollerPosition();
    m_intake.intakeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // if (first<=3000) {
      //   first++;
      // }
      // else {
        if (m_intake.getRollerSpeed()<=50 & Math.abs(m_intake.getRollerPosition()) > 1000){
          m_intake.setHasPiece(true);
        }
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  
    m_intake.intakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getHasPiece();
  }
}
