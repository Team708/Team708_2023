// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeRetraction extends CommandBase {

  Intake m_intake;
  double counts;
  boolean done;

  public IntakeRetraction(Intake m_intake, double counts) {
    this.m_intake = m_intake;
    this.counts = counts;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done=false;
    m_intake.setCamPosition(0);
    m_intake.setCamSpeed(IntakeConstants.kCamIntakeSpeed);
  }

  @Override
  public void execute(){
    if (!done){
      if (m_intake.getCamPosition() >= counts)
        done = true;

    }
  }

  // @Override
  // public void end(boolean interrupted){
  //   m_intake.setCamSpeed(0.0);
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (done){
        m_intake.setCamSpeed(0.0);
        return true;
      }
      else
      return false;
  }
}
