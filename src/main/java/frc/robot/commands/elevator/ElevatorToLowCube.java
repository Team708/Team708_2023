// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorToLowCube extends CommandBase {
  /** Creates a new ElevatorToPos. */

  private final Elevator m_elevator;
    private double distanceToGo;
  
    public ElevatorToLowCube(Elevator m_elevator) {
    this.m_elevator = m_elevator;  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPose(ElevatorConstants.kLowCubePose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToGo = m_elevator.getPose().getDistance(ElevatorConstants.kLowCubePose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceToGo <= ElevatorConstants.kPositionTolerance)
      return true;
    else
      return false;
  }
}
