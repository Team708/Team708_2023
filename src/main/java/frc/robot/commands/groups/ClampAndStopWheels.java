// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.grabberIntake.GrabberIntakeOff;
import frc.robot.commands.grabberIntake.GrabberIntakeRetraction;
import frc.robot.subsystems.intake.GrabberIntake;

public class ClampAndStopWheels extends ParallelCommandGroup {
  public ClampAndStopWheels(GrabberIntake m_intake) {
    addCommands(
      new GrabberIntakeOff(m_intake),
      new GrabberIntakeRetraction(m_intake, IntakeConstants.kCamClosedPose)
      );
  }
}
