package frc.robot.Utilities.DecisionTree;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class NodePath {

    private List<Node> path;
    private Elevator elevator;

    TrajectoryConfig config = new TrajectoryConfig(ElevatorConstants.kTrajConfigMaxVelocityMPS,
                                                   ElevatorConstants.kTrajConfigMaxAccelMPSS);

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
                return TrajectoryGenerator.generateTrajectory(poses, config);
            }
        } catch (IndexOutOfBoundsException | TrajectoryGenerationException e) {
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
