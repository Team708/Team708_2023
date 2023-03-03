// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.CANdleSystem;

public class SetTargetLEDs extends CommandBase {
  private CANdleSystem m_candleSystem;
  private int red, green, blue;

  /** Creates a new RequestCube. */
  public SetTargetLEDs(int red, int green, int blue) {
    this.red = red;
    this.green=green;
    this.blue = blue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_candleSystem.setColor(red, green, blue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
