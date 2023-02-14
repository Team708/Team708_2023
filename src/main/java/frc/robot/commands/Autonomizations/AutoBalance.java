// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomizations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveUntilBalanced;
import frc.robot.commands.drive.LockWheels;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoBalance extends SequentialCommandGroup {
  public AutoBalance(Drivetrain dr) {
    addCommands(
      new DriveUntilBalanced(dr),
      new LockWheels(dr)
    );
  }
}
