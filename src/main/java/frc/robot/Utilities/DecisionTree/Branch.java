
package frc.robot.Utilities.DecisionTree;

public class Branch {

    private Node a, b;

    public Branch(Node a, Node b){
        this.a = a;
        this.b = b;
        a.addNeighbor(b);
        b.addNeighbor(a);
    }

    public Node getA(){
        return a;
    }

    public Node getB(){
        return b;
    }

}
