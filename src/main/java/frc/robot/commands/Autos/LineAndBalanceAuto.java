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
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;

public class LineAndBalanceAuto extends SequentialCommandGroup {

  public LineAndBalanceAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, Intake m_intake, CANdleSystem m_candle) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DropConeAndBalance", maxSpeed, true);
    AutoFromPathPlanner path2 = new AutoFromPathPlanner(dr, "DropConeAndBalance2", maxSpeed, true);
    AutoFromPathPlanner path3 = new AutoFromPathPlanner(dr, "DropConeAndBalance3", maxSpeed, true);


    addCommands(
        new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())),

        //grabber head
        // new ElevatorToNode(m_elevator, Elevator.C),
        // new GrabberIntakeRetraction(m_intake, IntakeConstants.kCamOpenPose),
        
        //spinner head
        new ParallelDeadlineGroup(
          new ElevatorToNode(m_elevator, Elevator.C),
          new IntakeOn(m_intake)
        ),
        new WaitCommand(0.2),
        new IntakeOut(m_intake, m_candle).withTimeout(.2),
        new WaitCommand(0.2),
        new IntakeOff(m_intake),
        new ElevatorToNode(m_elevator, Elevator.B),
        //new IntakeRetraction(m_intake, IntakeConstants.kCamOpenPose),

        // new WaitCommand(3),
        path,
        
        
        // new ParallelDeadlineGroup(
        //   new ElevatorToNode(m_elevator, Elevator.A),
        //   new IntakeOn(m_intake)
        //   ),
          
        // new ParallelCommandGroup(
          path2,
          // new RaiseElevWhenPiece(m_intake, m_elevator).withTimeout(3)
          // ),
        
        new ElevatorToNode(m_elevator, Elevator.B),
        
        new WaitCommand(2),

        path3,

        new AutoBalance(dr)
      );
  }
}
