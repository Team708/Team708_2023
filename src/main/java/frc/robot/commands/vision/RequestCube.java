// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.CANdleSystem;

public class RequestCube extends CommandBase {
  private CANdleSystem m_candleSystem;
  /** Creates a new RequestCube. */
  public RequestCube(CANdleSystem m_candleSystem) {
    this.m_candleSystem = m_candleSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_candleSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_candleSystem.setColor(new Color8Bit(Color.kPurple));//114, 20, 181
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
