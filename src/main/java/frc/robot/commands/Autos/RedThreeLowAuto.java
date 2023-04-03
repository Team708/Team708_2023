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
import frc.robot.commands.Autonomizations.AutoBalance;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.intake.*;
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;


public class RedThreeLowAuto extends SequentialCommandGroup {

  public RedThreeLowAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, Intake m_intake, CANdleSystem m_candle) {
    AutoFromPathPlanner path1 = new AutoFromPathPlanner(dr, "RedThreeLow1", maxSpeed, true);
    AutoFromPathPlanner path2 = new AutoFromPathPlanner(dr, "RedThreeLow2", maxSpeed, true);
    AutoFromPathPlanner path3 = new AutoFromPathPlanner(dr, "RedThreeLow3", maxSpeed, true);
    AutoFromPathPlanner path4 = new AutoFromPathPlanner(dr, "RedThreeLow4", maxSpeed, true);
    AutoFromPathPlanner path5 = new AutoFromPathPlanner(dr, "RedThreeLow5", maxSpeed, true);

    addCommands(
      new InstantCommand(() -> dr.resetOdometry(path1.getInitialPose())),
      
      new ElevatorToNode(m_elevator, Elevator.K),
      new ParallelCommandGroup(
        path1,
        new RaiseElevWhenPiece(m_intake, m_elevator)
      ),

      path2,
      new WaitCommand(.2),

      new IntakeOut(m_intake, m_candle).withTimeout(.2),

      new ElevatorToNode(m_elevator, Elevator.A),
      
      // new WaitCommand(0.2),
      new ParallelCommandGroup(
        path3,
        new RaiseElevWhenPiece(m_intake, m_elevator)
      ),

      path4,

      new IntakeOut(m_intake, m_candle).withTimeout(.3),
      new ElevatorToNode(m_elevator, Elevator.B),

      new WaitCommand(0.2),

      new IntakeOff(m_intake),

      path5

      );
  }
    
}
