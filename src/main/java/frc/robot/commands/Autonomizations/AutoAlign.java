// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomizations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveUntilBalanced;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.TurnToZero;
import frc.robot.commands.vision.AlignWithReflectiveTape;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoAlign extends SequentialCommandGroup {
  public AutoAlign(Drivetrain dr) {
    addCommands(
      new TurnToZero(dr, new Pose2d(dr.getPose().getX(), dr.getPose().getY(), new Rotation2d(0)))
      // new AlignWithReflectiveTape(dr)
    );
  }
}
