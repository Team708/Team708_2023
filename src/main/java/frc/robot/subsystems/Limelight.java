package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CandleConstants;
import frc.robot.Utilities.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // private static Point2D[] points = new Point2D.Double[] {
    //         // (ty-angle,distance)
    //         new Point2D.Double(-22.7, 308.0), // 242
    //         new Point2D.Double(-20.0, 274.0), // 196
    //         new Point2D.Double(-17.5, 245.0), // 163
    //         new Point2D.Double(-15.0, 217.0), // 141
    //         new Point2D.Double(-12.5, 200.0), // 121
    //         new Point2D.Double(-10.0, 184.0), // 107
    //         new Point2D.Double(-5.0, 157.0), // 96
    //         new Point2D.Double(0.0, 136.0), // 85
    //         new Point2D.Double(5.0, 120.0), // 77
    //         new Point2D.Double(10.0, 106.0),
    //         new Point2D.Double(15.0, 94.0),
    //         new Point2D.Double(18.5, 86.0)
    //         //
    // };
    // private static LinearInterpolationTable distTable = new LinearInterpolationTable(points);

    //X
    public static double tx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    //Y
    public static double ty() {
        return table.getEntry("ty").getDouble(0.0);
    }

    //Valid
    public static boolean valid() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    //Area
    public static double ta() {
        return table.getEntry("ta").getDouble(0.0);
    }

    //ID of target apriltag
    public static int tid(){
        return (int)table.getEntry("tid").getInteger(1);
    }

    //Corners of blue box
    public static Number[] tcornxy(){
        return table.getEntry("tcornxy").getNumberArray(new Number[]{-1, -1, -1, -1});
    }
    
    // //Corners of blue box
    // public static void setLED(int led){
    //     table.getEntry("ledMode").setNumber(led);
    // }

    //Length of horizontal side of bb
    public static double thor(){
        return table.getEntry("thor").getInteger(-1);
    }

    //Length of vertical side of bb
    public static double tvert(){
        return table.getEntry("tvert").getInteger(-1);
    }

    public static double getDistance() {
        // final double tx = tx()*180.0/Math.PI;
        // final double tyAdj = (ty() -0.009742*tx*tx)/(0.00036*tx*tx+1.0); //New geometric correction function
        // final double distance = distTable.getOutput(tyAdj);
        //SmartDashboard.putNumber("Limelight ty", ty());
        // SmartDashboard.putNumber("LimelightDistance", distance);
        // return distance;
        return -1;
    }

    public static void enable() {
        table.getEntry("ledMode").setNumber(0);
    }

    public static void disable() {
        table.getEntry("ledMode").setNumber(1);
    }

    public static void setPipeline(int pipeline){
        table.getEntry("pipeline").setNumber(pipeline);
    }
    
}