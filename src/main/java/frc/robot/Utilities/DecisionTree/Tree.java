package frc.robot.Utilities.DecisionTree;

import java.util.ArrayList;
import java.util.HashMap;

public class Tree{

    private HashMap<String, Node> nodeMap;
    private ArrayList<Branch> branches = new ArrayList<Branch>();
                      //key,   value
    public Tree(HashMap<String, Node> nodeMap){
        this.nodeMap = nodeMap;
    }

    public void addBranch(Branch b) throws BranchExceptionError{
        if(nodeMap.containsValue(b.getA()) && nodeMap.containsValue(b.getB())){
            branches.add(b);
        }else throw new BranchExceptionError("Cannot add branch when both nodes do not exist");
    }

    public void printRelationships(){
        for(Branch b : branches){
            System.out.println("RELATIONSHIP ESTABLISHED BETWEEN " + b.getA().getIdentifier() 
            + " and " + b.getB().getIdentifier());
        }
    }
}
