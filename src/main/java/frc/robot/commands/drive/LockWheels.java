// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class LockWheels extends CommandBase {
  
  Drivetrain dr;
  SwerveModuleState[] preStates;

  public LockWheels(Drivetrain dr) {
    this.dr = dr;
    addRequirements(dr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    preStates = dr.getModuleStates();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] states = {
      new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), //LF
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //RF
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //LR
      new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)) //RR
    };
    dr.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dr.setModuleStates(preStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
