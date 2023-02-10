package frc.robot.Utilities.DecisionTree;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Node implements Comparable<Node>{

    private Translation2d position;
    private Pose2d pose;
    private double height;
    private double extend;
    private String identifier;
    private boolean isSafety;

    private double gScore = Double.MAX_VALUE; //set g-score to infinity by default
    private double hScore = Double.MAX_VALUE;
    private double finalScore = Double.MAX_VALUE;

    private Node parent = null;
    private LinkedList<Node> neighbors = new LinkedList<Node>();

    public Node(Translation2d position, String identifier, boolean isSafety){
        this.position = position;
        this.height = position.getY();
        this.extend = position.getX();
        this.identifier = identifier;
        this.isSafety = isSafety;
        this.pose = new Pose2d(position, new Rotation2d(0));
    }  

    public void reset(){
        gScore = 0;
        hScore = 0;
        finalScore = -1;
        parent = null;
    }

    public Translation2d getPosition(){
        return position;
    }

    public double getHeight(){
        return height;
    }

    public double getExtend(){
        return extend;
    }

    public Pose2d getPose(){
        return this.pose;
    }

    public boolean getIsSafety(){
        return isSafety;
    }

    public double getGScore() {
        return gScore;
    }

    public void setGScore(double g) {
        this.gScore = g;
    }

    public double getHScore() {
        return hScore;
    }

    public void setHScore(double h) {
        this.hScore = h;
    }

    public double getFinalScore(){
        return finalScore;
    }

    public void setFinalScore(double n){
        this.finalScore = n;
    }

    public void setParent(Node n) {
        this.parent = n;
    }

    public Node getParent() {
        return parent;
    }

    public void addNeighbor(Node neighbor){
        neighbors.add(neighbor);
    }

    public LinkedList<Node> getNeighbors(){
        return neighbors;
    }

    public String getIdentifier(){
        return identifier;
    }

    @Override
    public int compareTo(Node arg0) {
        return Double.compare(this.getFinalScore(), arg0.getFinalScore());
    }

}
