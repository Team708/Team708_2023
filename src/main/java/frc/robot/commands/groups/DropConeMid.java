// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.intake.IntakeRetraction;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;

public class DropConeMid extends SequentialCommandGroup {
  public DropConeMid(Elevator m_elevator, Intake m_intake) {
    addCommands(
      new ElevatorToNode(m_elevator, Elevator.F),
      new IntakeRetraction(m_intake, IntakeConstants.kCamOpenPose)
    );
  }
}
