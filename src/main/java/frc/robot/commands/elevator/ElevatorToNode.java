// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.List;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities.DecisionTree.*;
import frc.robot.subsystems.Elevator;

public class ElevatorToNode extends CommandBase {

  NodePathCalculator calculator;
  Node current, target;
  List<Translation2d> translations;
  Elevator elevator;
  boolean isFinished = false;
  Translation2d currPose;
  Trajectory trajectory;
  double i = 0;

  public ElevatorToNode(Elevator elevator, Node current, Node target) {
    this.elevator = elevator;
    this.current = current;
    this.target = target;
    calculator = new NodePathCalculator(elevator, elevator.getMap(), elevator.getElevatorBrances());
    addRequirements(elevator);
  }

  @Override
  public void initialize(){
    NodePath path = calculator.shortestPath(current, target);
    this.trajectory = path.translateToTrajectory();
    currPose = elevator.getPose();
    elevator.drawSimTrajectory(trajectory);
  }

  @Override
  public void execute(){
    // while(i <= trajectory.getTotalTimeSeconds()){
    //   Translation2d targetPose = trajectory.sample(i).poseMeters.getTranslation();
    //   elevator.setPose(targetPose);
    //   System.out.println(elevator.getPose().getDistance(targetPose));
    //   if(elevator.getPose().getDistance(targetPose) <= 0.005){
    //     i += 0.05;
    //     System.out.println(elevator.getPose().getDistance(trajectory.sample(i).poseMeters.getTranslation()));
    //   }
    //   // else{
    //   //   elevator.setPose(trajectory.sample(i).poseMeters.getTranslation());
    //   // }
    //   SmartDashboard.putNumber("trajectory",trajectory.sample(i).poseMeters.getTranslation().getX());
      //System.out.println(elevator.getPose().getDistance(trajectory.sample(i).poseMeters.getTranslation()));
    Translation2d targetPose = trajectory.sample(i).poseMeters.getTranslation();
    elevator.setPose(targetPose);
    i += 0.05;
    if(elevator.getPose().getDistance(targetPose) <= 0.000005){
      if(i >= trajectory.getTotalTimeSeconds()) i = trajectory.getTotalTimeSeconds();
    }
    System.out.println(i);
    // System.out.println(elevator.getPose().getDistance(trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation()));
      // else{
      //   elevator.setPose(trajectory.sample(i).poseMeters.getTranslation());
      // }
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("END!");
  }

  @Override
  public boolean isFinished(){
    return elevator.getPose().getDistance(
      trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation()) 
    < Constants.ElevatorConstants.kPositionTolerance;
  }
}
