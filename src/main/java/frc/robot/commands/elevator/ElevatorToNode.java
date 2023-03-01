// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Utilities.DecisionTree.*;
import frc.robot.subsystems.Elevator;

public class ElevatorToNode extends CommandBase {

  Node current, target;
  List<Translation2d> translations;
  Elevator elevator;
  boolean isFinished = false;
  Trajectory trajectory;
  double i = 0;

  public ElevatorToNode(Elevator elevator, Node target) {
    this.elevator = elevator;
    this.target = target;
    addRequirements(elevator);
  }

  @Override
  public void initialize(){
    // this.current = elevator.getElevatorNode();
    this.current = elevator.getClosestNode();
    SmartDashboard.putString("CLOSEST NODE", elevator.getClosestNode().getIdentifier());
    elevator.setPose(this.current.getPosition());
    NodePath path = NodePathCalculator.shortestPath(elevator.getElevatorTree(), elevator, current, target);
    this.trajectory = path.translateToTrajectory();
    if(this.trajectory != null && elevator.getSim() != null) elevator.getSim().drawTrajectory(trajectory);
    i = 0;
  }

  @Override
  public void execute(){
    if(this.trajectory == null) return;
    Translation2d targetPose = trajectory.sample(i).poseMeters.getTranslation();
    elevator.setPose(targetPose);
    i += GlobalConstants.kLoopTime;
    if(i >= trajectory.getTotalTimeSeconds()) i = trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted){
    // Node temp = elevator.getClosestNode();
    // elevator.setPose(temp.getPosition());
    // elevator.setElevatorNode(temp);
    elevator.setElevatorNode(target);
  }

  @Override
  public boolean isFinished(){
    if(this.trajectory == null) return true;
    
    return (elevator.getPose().getDistance(
              trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation()) 
            < Constants.ElevatorConstants.kPositionTolerance) ||
              Math.abs(OI.getOperatorLeftY()) > 0.1 ||
              Math.abs(OI.getOperatorRightX()) > 0.1;
    // return true;
  }
}
