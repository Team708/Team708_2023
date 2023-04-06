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
import frc.robot.commands.intake.*;
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;


public class LeftDriveToPieceAuto extends SequentialCommandGroup {

  public LeftDriveToPieceAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, Intake m_intake, CANdleSystem m_candle) {
    AutoFromPathPlanner path1 = new AutoFromPathPlanner(dr, "LeftSideDriveToPiece2", maxSpeed, true);
    AutoFromPathPlanner path2 = new AutoFromPathPlanner(dr, "LeftSideDriveToPiece", maxSpeed, true);
    AutoFromPathPlanner path3 = new AutoFromPathPlanner(dr, "turn180_2", maxSpeed, true);
    AutoFromPathPlanner path4 = new AutoFromPathPlanner(dr, "LeftSideDriveToPiece3", maxSpeed, true);

    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path1.getInitialPose())),
      
      new ParallelDeadlineGroup(
        new ElevatorToNode(m_elevator, Elevator.C)/*.withTimeout(3.3)*/,
        new IntakeOn(m_intake)
      ),
      new WaitCommand(0.2),
      new IntakeOut(m_intake, m_candle).withTimeout(.2),
      new WaitCommand(0.2),
      // new IntakeOff(m_intake),
      // new WaitCommand(0.2),
      new ElevatorToNode(m_elevator, Elevator.A),
      new ParallelCommandGroup(
        path1,
        new RaiseElevWhenPiece(m_intake, m_elevator)
      ),

      new WaitCommand(.2),
      path2,
      new ElevatorToNode(m_elevator, Elevator.D),
      new IntakeOut(m_intake, m_candle).withTimeout(.2),
      // new WaitCommand(0.2),
      // new IntakeOff(m_intake),
      // new WaitCommand(0.2),
      new ElevatorToNode(m_elevator, Elevator.A),

      new ParallelCommandGroup(
        path3,
        new RaiseElevWhenPiece(m_intake, m_elevator)
      ),

      path4,

      new WaitCommand(0.2),
      new InstantCommand(() -> dr.resetOdometry(path1.getInitialPose())),
      new WaitCommand(0.2),
      new InstantCommand(() -> dr.resetOdometry(path1.getInitialPose()))

      );
  }
    
}
