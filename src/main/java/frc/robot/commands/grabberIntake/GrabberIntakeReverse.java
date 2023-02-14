// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabberIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.GrabberIntake;

public class GrabberIntakeReverse extends CommandBase {

  GrabberIntake m_intake;

  public GrabberIntakeReverse(GrabberIntake m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
