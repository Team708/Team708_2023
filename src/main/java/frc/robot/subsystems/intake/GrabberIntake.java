package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CurrentLimit;

public class GrabberIntake extends SubsystemBase{
    
    private CANSparkMax m_intakeMotor;
    private RelativeEncoder m_intakeEncoder;

    private boolean isOpen = false;

    public GrabberIntake(){
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotor.setSmartCurrentLimit(CurrentLimit.kIntake);
        m_intakeMotor.setInverted(false);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeEncoder.setPositionConversionFactor(IntakeConstants.kCamGearRatio);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void setCamPosition(double rotations){
        m_intakeEncoder.setPosition(rotations);
    }

    public void openIntake(){
        if(!isOpen){
            setCamPosition(IntakeConstants.kCamOpenPose);
            this.isOpen = true;
        }
    }

    public void closeIntake(){
        if(isOpen){
            setCamPosition(IntakeConstants.kCamClosedPose);
            this.isOpen = false;
        }
    }

    public boolean getIsOpen(){
        return isOpen;
    }

    public void setIsOpen(boolean isOpen){
        this.isOpen = isOpen;
    }

}
