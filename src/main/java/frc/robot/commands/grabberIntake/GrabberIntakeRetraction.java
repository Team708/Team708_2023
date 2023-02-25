// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabberIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.GrabberIntake;

public class GrabberIntakeRetraction extends CommandBase {

  GrabberIntake m_intake;
  double counts;
  boolean greater = false;

  public GrabberIntakeRetraction(GrabberIntake m_intake, double counts) {
    this.m_intake = m_intake;
    this.counts = counts;
    // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(counts > m_intake.getCamPosition()){
      m_intake.setCamSpeed(IntakeConstants.kCamIntakeSpeed);
      greater = true;
    }else{
      m_intake.setCamSpeed(-IntakeConstants.kCamIntakeSpeed);
      greater = false;
    }
  }

  @Override
  public void execute(){

  }

  @Override
  public void end(boolean interrupted){
    m_intake.setCamSpeed(0.0);
    // m_intake.setCamPosition(counts);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(greater){
      return m_intake.getCamPosition() >= counts;
    }else{
      return m_intake.getCamPosition() <= counts;
    }
  }
}
