// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeRetraction;
import frc.robot.subsystems.intake.Intake;

public class ClampAndStopWheels extends ParallelCommandGroup {
  public ClampAndStopWheels(Intake m_intake) {
    addCommands(
      new IntakeOff(m_intake),
      new IntakeRetraction(m_intake, IntakeConstants.kCamOpenPose)
      );
  }
}
