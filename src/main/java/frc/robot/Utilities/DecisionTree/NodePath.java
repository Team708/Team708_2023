package frc.robot.Utilities.DecisionTree;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.elevator.ElevatorToGroundSafe;
import frc.robot.commands.elevator.ElevatorToHighCone;
import frc.robot.commands.elevator.ElevatorToHighCube;
import frc.robot.commands.elevator.ElevatorToHighSafe;
import frc.robot.commands.elevator.ElevatorToLowCone;
import frc.robot.commands.elevator.ElevatorToLowCube;
import frc.robot.commands.elevator.ElevatorToLowSafe;
import frc.robot.commands.elevator.ElevatorToMidSafe;
import frc.robot.commands.elevator.ElevatorToStart;
import frc.robot.subsystems.Elevator;

public class NodePath {

    private List<Node> path;
    private Elevator elevator;

    // TODO MAKE CONSTANTS
    TrajectoryConfig config = new TrajectoryConfig(1, 2);

    public NodePath(List<Node> path, Elevator elevator) {
        this.path = path;
        this.elevator = elevator;
    }

    public List<Translation2d> getTranslationPath() {
        ArrayList<Translation2d> ret = new ArrayList<Translation2d>();
        path.stream().forEach(i -> ret.add(i.getPosition()));
        return ret;
    }

    /**
     * @deprecated
     * @return A list of commands that sort of represent the elevator's movements
     */
    public List<Command> translateToCommandPath() {
        ArrayList<Command> ret = new ArrayList<Command>();
        for (Node n : path) {
            if (n.getIdentifier().equals("GROUND_PICKUP")) {
                ret.add(new ElevatorToGround(elevator));
            } else if (n.getIdentifier().equals("GROUND_SAFE")) {
                ret.add(new ElevatorToGroundSafe(elevator));
            } else if (n.getIdentifier().equals("HIGH_CONE")) {
                ret.add(new ElevatorToHighCone(elevator));
            } else if (n.getIdentifier().equals("HIGH_CUBE")) {
                ret.add(new ElevatorToHighCube(elevator));
            } else if (n.getIdentifier().equals("HIGH_SAFE")) {
                ret.add(new ElevatorToHighSafe(elevator));
            } else if (n.getIdentifier().equals("LOW_CONE")) {
                ret.add(new ElevatorToLowCone(elevator));
            } else if (n.getIdentifier().equals("LOW_CUBE")) {
                ret.add(new ElevatorToLowCube(elevator));
            } else if (n.getIdentifier().equals("LOW_SAFE")) {
                ret.add(new ElevatorToLowSafe(elevator));
            } else if (n.getIdentifier().equals("MID_SAFE")) {
                ret.add(new ElevatorToMidSafe(elevator));
            } else if (n.getIdentifier().equals("START")) {
                ret.add(new ElevatorToStart(elevator));
            }
        }
        return ret;
    }

    /**
     * Translates the {@linkNodePath} to a trajectory
     * 
     * @return A generated trajectory
     */
    public Trajectory translateToTrajectory() {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        path.stream().forEach(i -> poses.add(i.getPose()));
        try {
            if (poses.size() > 1) {
                ArrayList<Translation2d> translations = new ArrayList<Translation2d>();
                for (int i = 1; i < poses.size() - 1; i++) {
                    translations.add(poses.get(i).getTranslation());
                }
                SmartDashboard.putString("CURRENT NODE", path.get(0).getIdentifier());
                double startAngle = getPredictedBeginningAngle(poses.get(0), poses.get(1));
                double endAngle = getPredictedBeginningAngle(poses.get(poses.size() - 2), poses.get(poses.size() - 1));
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(poses.get(0).getTranslation(), new Rotation2d(startAngle)),
                        translations,
                        new Pose2d(poses.get(poses.size() - 1).getTranslation(), new Rotation2d(endAngle)),
                        config); // TODO Change trajectory config
            } else {
                System.out.println("ENCOUNTERED");
                return TrajectoryGenerator.generateTrajectory(poses, config);
            }
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
    }

    public double getPredictedBeginningAngle(Pose2d start, Pose2d next) {
        double diffX = next.getX() - start.getX();
        double diffY = next.getY() - start.getY();
        return Math.atan2(diffY, diffX);
    }

    public void printPath() {
        for (Node n : path) {
            System.out.print(n.getIdentifier() + ", ");
        }
    }
}
