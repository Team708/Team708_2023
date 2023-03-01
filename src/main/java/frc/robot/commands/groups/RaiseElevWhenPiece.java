// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorFromGround;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;

public class RaiseElevWhenPiece extends SequentialCommandGroup {
  public RaiseElevWhenPiece(Intake m_intake, Elevator m_elevator) {
    addCommands(
      new ElevatorToNode(m_elevator, Elevator.A),

      new IntakeOn(m_intake),
      new ElevatorFromGround(m_elevator, Elevator.B, m_intake)
      );
  }
}
