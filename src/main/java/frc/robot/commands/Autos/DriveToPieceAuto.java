// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.Autonomizations.AutoBalance;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.grabberIntake.GrabberIntakeOn;
import frc.robot.commands.grabberIntake.GrabberIntakeOut;
import frc.robot.commands.grabberIntake.GrabberIntakeRetraction;
import frc.robot.commands.groups.DropConeHigh;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.GrabberIntake;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.drive.InvertDriveCommand;
import frc.robot.commands.drive.ResetDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToPieceAuto extends SequentialCommandGroup {

  public DriveToPieceAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, GrabberIntake m_intake) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DriveToPiece", maxSpeed, true);
    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())),
      
      new ParallelDeadlineGroup(
        new ElevatorToNode(m_elevator, Elevator.C),
        new GrabberIntakeOn(m_intake)
      ),
      new GrabberIntakeOut(m_intake).withTimeout(.2),
      
      new ElevatorToNode(m_elevator, Elevator.A),
      new GrabberIntakeOn(m_intake),
      path,
      new ElevatorToNode(m_elevator, Elevator.D),
      new GrabberIntakeOut(m_intake).withTimeout(.2)

      );
  }
    
}
