// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class ResetDriveCommand extends CommandBase {
  
  private final Drivetrain dr;
  private final Rotation2d orientation;

  public ResetDriveCommand(Drivetrain dr, Rotation2d orientation) {
    this.dr = dr;
    this.orientation = orientation;

    addRequirements(dr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dr.resetOdometry(new Pose2d(0.0,0.0,orientation));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
