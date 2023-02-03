// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities.DecisionTree.*;
import frc.robot.subsystems.Elevator;

public class CalculateElevatorPath extends CommandBase {

  NodePathCalculator calculator;
  Node current, target;

  public CalculateElevatorPath(Elevator elevator, Node current, Node target) {
    this.current = current;
    this.target = target;
    calculator = new NodePathCalculator(elevator.getMap(), elevator.getElevatorBrances());
  }

  @Override
  public void initialize() {
    NodePath path = calculator.shortestPath(current, target);
    System.out.println("Path from " + current.getIdentifier() + " to " + target.getIdentifier());
    path.printPath();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
