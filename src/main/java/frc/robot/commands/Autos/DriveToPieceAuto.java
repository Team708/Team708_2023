// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.intake.*;
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;


public class DriveToPieceAuto extends SequentialCommandGroup {

  public DriveToPieceAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, Intake m_intake) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DriveToPiece", maxSpeed, true);
    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())),
      
      new ParallelDeadlineGroup(
        new ElevatorToNode(m_elevator, Elevator.C),
        new IntakeOn(m_intake)
      ),
      new IntakeOut(m_intake).withTimeout(.2),
      
      // new ElevatorToNode(m_elevator, Elevator.A),
      new RaiseElevWhenPiece(m_intake, m_elevator),
      path,
      new ElevatorToNode(m_elevator, Elevator.D),
      new IntakeOut(m_intake).withTimeout(.2)

      );
  }
    
}
