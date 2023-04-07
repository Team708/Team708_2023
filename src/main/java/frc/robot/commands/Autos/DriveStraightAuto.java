// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;

public class DriveStraightAuto extends SequentialCommandGroup {

  public DriveStraightAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, Intake m_intake, CANdleSystem m_candle) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DriveStraight", maxSpeed, true);
    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())),

      new ParallelDeadlineGroup(
        new ElevatorToNode(m_elevator, Elevator.C)/*.withTimeout(3.3)*/,
        new IntakeOn(m_intake)
      ),
      new WaitCommand(1.0),
      new IntakeOut(m_intake, m_candle).withTimeout(.2),
      
      new WaitCommand(1.0),
      new ElevatorToNode(m_elevator, Elevator.B),

      new IntakeOff(m_intake),
      new WaitCommand(0.2),

      path
      );
  }
}
