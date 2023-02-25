package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class InvertDriveCommand extends CommandBase {

    private final Drivetrain m_DriveSubsystem;

    public InvertDriveCommand(Drivetrain subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.invertDrive();;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
