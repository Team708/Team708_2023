// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomizations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.vision.ActivateTape;
import frc.robot.commands.vision.AlignWithReflectiveTape;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.CANdleSystem;

public class AutoAlignTape extends SequentialCommandGroup {
  public AutoAlignTape(Drivetrain dr, CANdleSystem candle) {
    addCommands(
      new ActivateTape(),
      new AlignWithReflectiveTape(dr, candle)
    );
  }
}
