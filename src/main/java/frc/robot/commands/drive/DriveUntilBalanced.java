// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PigeonTwo;

public class DriveUntilBalanced extends CommandBase {
  
  Drivetrain dr;
  PigeonTwo pigeonInstance;
  double initialDirection = 0;
  SwerveModuleState[] preState;
  double lastRoll = 0;
  double currRoll = 0;

  public DriveUntilBalanced(Drivetrain dr) {
    this.dr = dr;
    pigeonInstance = PigeonTwo.getInstance();
    preState = dr.getModuleStates();
    addRequirements(dr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Math.abs(pigeonInstance.getRoll().getDegrees()) > 2){ //Just as a precaution
        initialDirection = Math.signum(pigeonInstance.getRoll().getDegrees());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] states;
    lastRoll = currRoll;
    currRoll = pigeonInstance.getRoll().getDegrees();
    if(Math.abs(currRoll) < 5){
      states = new SwerveModuleState[]{
          new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), //LF
          new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //RF
          new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //LR
          new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)) //RR
      };
      dr.setModuleStates(states);
      // dr.setModuleStates(preState);
    }else{
    // while(Math.abs(pigeonInstance.getRoll().getDegrees()) > 0.5){
        if(initialDirection == 1.0){
            if(Math.signum(pigeonInstance.getRoll().getDegrees()) == 1){
                states = new SwerveModuleState[]{
                    new SwerveModuleState(0.25, new Rotation2d(0)), //LF
                    new SwerveModuleState(0.25, new Rotation2d(0)), //RF
                    new SwerveModuleState(0.25, new Rotation2d(0)), //LR
                    new SwerveModuleState(0.25, new Rotation2d(0)) //RR
                  };
            }else{
                states = new SwerveModuleState[]{
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //LF
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //RF
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //LR
                    new SwerveModuleState(-0.25, new Rotation2d(0)) //RR
                };
            }
        }else if(initialDirection == -1.0){
            if(Math.signum(pigeonInstance.getRoll().getDegrees()) == 1){
                states = new SwerveModuleState[]{
                    new SwerveModuleState(0.25, new Rotation2d(0)), //LF
                    new SwerveModuleState(0.25, new Rotation2d(0)), //RF
                    new SwerveModuleState(0.25, new Rotation2d(0)), //LR
                    new SwerveModuleState(0.25, new Rotation2d(0)) //RR
                  };
            }else{
                states = new SwerveModuleState[]{
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //LF
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //RF
                    new SwerveModuleState(-0.25, new Rotation2d(0)), //LR
                    new SwerveModuleState(-0.25, new Rotation2d(0)) //RR
                };
            }
        }else{
            states = new SwerveModuleState[]{
                new SwerveModuleState(0, new Rotation2d(0)), //LF
                new SwerveModuleState(0, new Rotation2d(0)), //RF
                new SwerveModuleState(0, new Rotation2d(0)), //LR
                new SwerveModuleState(0, new Rotation2d(0)) //RR
            };
        }
        dr.setModuleStates(states);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveModuleState[] states;
    states = new SwerveModuleState[]{
      new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), //LF
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //RF
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), //LR
      new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)) //RR
    };
    dr.setModuleStates(states);
    // dr.setModuleStates(preState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(currRoll) < 2;
    return false;
  }
}
