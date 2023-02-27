// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.grabberIntake.GrabberIntakeRetraction;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.GrabberIntake;

public class DropConeHigh extends SequentialCommandGroup {
  public DropConeHigh(Elevator m_elevator, GrabberIntake m_intake) {
    addCommands(
      new ElevatorToNode(m_elevator, Elevator.C),
      new GrabberIntakeRetraction(m_intake, IntakeConstants.kCamOpenPose),
      new ElevatorToNode(m_elevator, Elevator.B)

    );
  }
}
