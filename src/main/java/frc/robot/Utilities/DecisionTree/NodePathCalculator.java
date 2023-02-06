package frc.robot.Utilities.DecisionTree;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import frc.robot.subsystems.Elevator;

public class NodePathCalculator {

    public LinkedList<Node> finalPath = new LinkedList<>();
    public Tree nodeTree;
    private Elevator elevator;

    public NodePathCalculator(Elevator elevator, HashMap<String, Node> nodes, Branch... branches){
        this.elevator = elevator;
        //Generate tree
        nodeTree = new Tree(nodes);
        for (Branch branch : branches) {
            try{
                nodeTree.addBranch(branch);
            }catch(BranchExceptionError e){
                e.printStackTrace();
            }
        }
        // nodeTree.printRelationships();
    }

    public NodePath shortestPath(Node start, Node goal){
        finalPath.clear();
        List<Node> open = new ArrayList<Node>(); //Open Nodes
        List<Node> closed = new ArrayList<Node>(); //Closed Nodes
        open.add(start); //Add the start node to open
        while(true){
            Node current = open.get(0);
            for(Node n : open){
                if(n.getFinalScore() < current.getFinalScore()){
                    current = n;
                }
            }
            open.remove(current);
            closed.add(current);
            if(current.getIdentifier() == goal.getIdentifier()){
                search(current, start, goal);
                break;
            }
            List<Node> neighbors = calcCloseNodes(current);
            for(Node n : neighbors){
                if(n != null){
                    double newPath = n.getGScore() /*+ calculateScores(n, current)*/;
                    if(newPath < n.getGScore() || !open.contains(n)){
                        n.setFinalScore(newPath + getDistance(start, n));
                        n.setParent(current);
                        if(!open.contains(n)){
                            open.add(n);
                        }
                    }
                }
            }
        }
        LinkedList<Node> tempFinalPath = new LinkedList<Node>();
        for(int i = finalPath.size() - 1; i >= 0; i--){
            tempFinalPath.add(finalPath.get(i));
        }
        return new NodePath(tempFinalPath, elevator);
    }

    private void search(Node n, Node start, Node goal){
        try{
            if(n.getIdentifier().equals(goal.getIdentifier())){
                finalPath.add(n);
            }
            if(!n.getIdentifier().equals(start.getIdentifier())) {
                finalPath.add(n.getParent());
                search(n.getParent(), start, goal);
            }
        }catch(NullPointerException e){
            e.printStackTrace();
        }
    }

    private LinkedList<Node> calcCloseNodes(Node n){
        LinkedList<Node> neighbors = n.getNeighbors();
        for(Node curr : neighbors){
            curr.setGScore(n.getGScore() + 1);
            curr.setHScore(getDistance(curr, n));
            curr.setFinalScore(curr.getGScore() + curr.getHScore());
        }
        return neighbors;
    }

    private double getDistance(Node a, Node b){
        return Math.sqrt(
            Math.pow((a.getPose().getX() - b.getPose().getX()), 2) +
            Math.pow((a.getPose().getY() - b.getPose().getY()), 2)
        ); 
    }

    // private double calculateScores(Node a, Node b){
    //     int add;
    //     if(a.getPose().getX() == b.getPose().getX() || a.getPose().getY() == b.getPose().getY()){
    //         add = 10;
    //     }else{
    //         add = 14;
    //     }
    //     return add;
    // }

}
