package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CurrentLimit;

public class GrabberIntake extends SubsystemBase{
    
    private CANSparkMax m_intakeMotor;
    private CANSparkMax m_clampMotor;
    private RelativeEncoder m_clampEncoder;

    private boolean isOpen = false;
    private boolean isReversed = false;

    PIDController controller = new PIDController(1, 0, 0); //SET AS CONSTANTS

    public GrabberIntake(){
        m_clampMotor = new CANSparkMax(IntakeConstants.kClampMotorID, MotorType.kBrushless);
        m_clampMotor.setSmartCurrentLimit(CurrentLimit.kIntake);
        m_clampMotor.setInverted(false);
        m_clampMotor.setIdleMode(IdleMode.kBrake);

        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotor.setSmartCurrentLimit(CurrentLimit.kIntake);
        m_intakeMotor.setInverted(false);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        m_clampEncoder = m_clampMotor.getEncoder();
        m_clampEncoder.setPositionConversionFactor(IntakeConstants.kCamGearRatio);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void setCamPosition(double angle){
        // double output = controller.calculate(m_clampEncoder.getPosition(), angle);
        // m_clampMotor.setVoltage(output);
        m_clampMotor.set(angle);
    }

    public double getCamPosition(){
        return m_clampEncoder.getPosition();
    }

    public void openClamp(){
        if(!isOpen){
            setCamPosition(IntakeConstants.kCamOpenPose);
            this.isOpen = true;
        }
    }

    public void closeClamp(){
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

    public void intakeOn(){
        this.m_intakeMotor.set(IntakeConstants.kCamIntakeSpeed);
        isReversed = false;
    }

    public void intakeOff(){
        this.m_intakeMotor.set(0);
    }

    public void intakeReverse(){
        this.m_intakeMotor.set(-IntakeConstants.kCamIntakeSpeed);
        isReversed = true;
    }

    public boolean getIsReversed(){
        return isReversed;
    }

    public void sendToDashboard(){
        SmartDashboard.putNumber("Intake Encoder Val", getCamPosition());
    }

}
