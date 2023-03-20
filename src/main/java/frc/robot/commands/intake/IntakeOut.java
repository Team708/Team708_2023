// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;

public class IntakeOut extends CommandBase {

  Intake m_intake;
  CANdleSystem m_candle;

  public IntakeOut(Intake m_intake, CANdleSystem m_candle) {
    this.m_intake = m_intake;
    this.m_candle = m_candle;
    // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeOut();
    m_candle.setColor(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intake.intakeOff();
      // m_intake.setHasPiece(false);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
