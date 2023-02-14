// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToPieceAuto extends SequentialCommandGroup {

  public DriveToPieceAuto(Drivetrain dr, double maxSpeed) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DriveToPiece", maxSpeed, true);
    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())),
      path
      );
  }
}
