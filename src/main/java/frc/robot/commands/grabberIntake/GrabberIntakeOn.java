// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabberIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.GrabberIntake;

public class GrabberIntakeOn extends CommandBase {

  GrabberIntake m_intake;
  Elevator m_elevator;

  public GrabberIntakeOn(GrabberIntake m_intake, Elevator m_elevator) {
    this.m_intake = m_intake;
    this.m_elevator = m_elevator;

    // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getRollerSpeed()<=100){
      if (m_elevator.getAtGroundPickup()==false){
        // new InstantCommand(() -> m_elevator.ElevatorToNode(m_elevator, Elevator.B));
      }
      m_elevator.setAtGroundPickup(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
