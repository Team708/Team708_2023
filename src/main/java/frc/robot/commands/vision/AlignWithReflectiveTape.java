package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.CANdleSystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

  /**
   * Implements a DriveByController command which extends the CommandBase class
   */
public class AlignWithReflectiveTape extends CommandBase {
  
    private final Drivetrain m_robotDrive;
    private final CANdleSystem m_candleSystem;
    private final SwerveModuleState[] preStates;

    public AlignWithReflectiveTape(Drivetrain drive, CANdleSystem candle) {
        m_robotDrive = drive;
        m_candleSystem = candle;
        preStates = m_robotDrive.getModuleStates();
        addRequirements(m_robotDrive, m_candleSystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        SwerveModuleState[] states;
        // while(Math.abs(Limelight.tx()) > 0.1){
            
            if(Math.signum(Limelight.tx()) == 1.0){
               m_candleSystem.setColor(0,0,255);
                states = new SwerveModuleState[]{
                    new SwerveModuleState(-Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(-Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(-Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(-Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2))
                };
            }else if(Math.signum(Limelight.tx()) == -1.0){
                m_candleSystem.setColor(0,0,255);
                states = new SwerveModuleState[]{
                    new SwerveModuleState(Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(Constants.VisionConstants.kLineupSpeed, new Rotation2d(Math.PI / 2))
                };
            }else{
                if(!Limelight.valid()){
                    m_candleSystem.setColor(255,0,0);
                }else{
                    m_candleSystem.setColor(0,255,0);
                }
                states = new SwerveModuleState[]{
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 2))
                };
            }
            m_robotDrive.setModuleStates(states);
        // }
    }

    @Override
    public void end(boolean interrupted){
        m_robotDrive.setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, preStates[0].angle),
            new SwerveModuleState(0, preStates[1].angle),
            new SwerveModuleState(0, preStates[2].angle),
            new SwerveModuleState(0, preStates[3].angle)
        });
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Limelight.tx()) < Constants.VisionConstants.kLineupAccuracy;
    }

}
