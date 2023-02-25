// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.Autonomizations.AutoBalance;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.groups.DropConeHigh;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.GrabberIntake;

public class LineAndBalanceAuto extends SequentialCommandGroup {

  public LineAndBalanceAuto(Drivetrain dr, double maxSpeed, Elevator m_elevator, GrabberIntake m_intake) {
    AutoFromPathPlanner path = new AutoFromPathPlanner(dr, "DropConeAndBalance", maxSpeed, true);
    addCommands(
        new InstantCommand(() -> dr.resetOdometry(path.getInitialPose())).withTimeout(5),
        new DropConeHigh(m_elevator, m_intake).withTimeout(5),
        new ParallelCommandGroup(
            path,
            new ElevatorToNode(m_elevator, Elevator.B)
        ).withTimeout(5),
        new AutoBalance(dr)
      );
  }
}
