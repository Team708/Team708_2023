package frc.robot.Utilities.DecisionTree;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import frc.robot.subsystems.Elevator;

public class NodePathCalculator {

    private static LinkedList<Node> finalPath = new LinkedList<>();

    // public NodePathCalculator(Elevator elevator, HashMap<String, Node> nodes, Branch... branches){
    //     this.elevator = elevator;
    //     //Generate tree
        
    // }

    public static NodePath shortestPath(Tree nodeTree, Elevator elevator, Node start, Node goal){
        nodeTree.getNodes().stream().forEach(i -> i.reset());
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
                        n.setFinalScore(newPath + Math.abs(getDistance(start, n)));
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
        // tempFinalPath.stream().forEach(i -> System.out.println(i.getIdentifier()));
        return new NodePath(tempFinalPath, elevator);
    }

    private static void search(Node n, Node start, Node goal){
        try{
            if(n.getIdentifier().equals(goal.getIdentifier())){
                finalPath.add(n);
            }
            if(!n.getIdentifier().equals(start.getIdentifier())) {
                if(!finalPath.contains(n.getParent())){
                    finalPath.add(n.getParent());
                    // System.out.println(n.getIdentifier() + " -> " + n.getParent().getIdentifier());
                    search(n.getParent(), start, goal);
                }
            }
        }catch(NullPointerException | StackOverflowError e){
            // e.printStackTrace();
        }
    }

    private static LinkedList<Node> calcCloseNodes(Node n){
        LinkedList<Node> neighbors = n.getNeighbors();
        for(Node curr : neighbors){
            curr.setGScore(n.getGScore() + 1);
            curr.setHScore(getDistance(curr, n));
            curr.setFinalScore(curr.getGScore() + curr.getHScore());
        }
        return neighbors;
    }

    private static double getDistance(Node a, Node b){
        return Math.sqrt(
            Math.pow((Math.abs(a.getPose().getX() - b.getPose().getX())), 2) +
            Math.pow((Math.abs(a.getPose().getY() - b.getPose().getY())), 2)
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
