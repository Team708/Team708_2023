package frc.robot.Utilities.DecisionTree;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;

public class NodePath {

    List<Node> path;

    public NodePath(List<Node> path){
        this.path = path;
    }

    public Trajectory translateToTrajectory(){
        return null;
    }

    public void printPath(){
        for(Node n : path){
            System.out.print(n.getIdentifier() + ", ");
        }
    }
}
