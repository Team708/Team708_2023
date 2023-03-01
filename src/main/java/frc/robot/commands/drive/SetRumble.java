package frc.robot.commands.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.OI;

public class SetRumble extends CommandBase {


  public SetRumble() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}