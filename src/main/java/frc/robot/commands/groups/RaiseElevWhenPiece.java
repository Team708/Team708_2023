// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorFromGround;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;

public class RaiseElevWhenPiece extends ParallelCommandGroup {
  public RaiseElevWhenPiece(Intake m_intake, Elevator m_elevator) {
    addCommands(
      new IntakeOn(m_intake, m_elevator),
      new ElevatorFromGround(m_elevator, Elevator.B, m_intake)
      );
  }
}
