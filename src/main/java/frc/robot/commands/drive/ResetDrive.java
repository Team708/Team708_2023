package frc.robot.commands.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;

public class ResetDrive extends CommandBase {

  private final Drivetrain dr;
  private final Rotation2d orientation;

  public ResetDrive(Drivetrain dr, Rotation2d orientation) {
    this.dr = dr;
    this.orientation = orientation;

    addRequirements(dr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dr.resetOdometry(new Pose2d(0.0,0.0,orientation));
    OI.driverController.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}