
package frc.robot.Utilities.DecisionTree;

public class Branch {

    private Node a, b;
    private double weight = -1;

    public Branch(Node a, Node b){
        this.a = a;
        this.b = b;
        this.weight = a.getPosition().getDistance(b.getPosition());
        a.addNeighbor(b);
        b.addNeighbor(a);
    }

    public Node getA(){
        return a;
    }

    public Node getB(){
        return b;
    }

    public double getWeight(){
        return weight;
    }

}
