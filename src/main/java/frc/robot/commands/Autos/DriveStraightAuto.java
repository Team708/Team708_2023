// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.drive.ResetDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveStraightAuto extends SequentialCommandGroup {

  public DriveStraightAuto(Drivetrain dr, double maxSpeed) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DriveStraight", maxSpeed, true);
    addCommands(
      new ResetDriveCommand(dr, new Rotation2d()),
      path
      );
  }
}
