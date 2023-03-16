package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CurrentLimit;

public class Intake extends SubsystemBase{
    
    private CANSparkMax m_intakeMotor;
    private CANSparkMax m_clampMotor;
    private RelativeEncoder m_clampEncoder;
    private RelativeEncoder m_intakeEncoder;
    
    private boolean isOpen = false;
    private boolean isReversed = false;
    private boolean hasPiece = false;

    private DigitalInput m_dIOSensor;

    public Intake(DigitalInput m_dIOSensor){

        // this.m_dIOSensor = m_dIOSensor;

        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotor.setSmartCurrentLimit(CurrentLimit.kIntake);
        m_intakeMotor.setInverted(false);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        // m_clampEncoder = m_clampMotor.getEncoder();

        m_intakeEncoder.setVelocityConversionFactor(100);
        m_intakeEncoder.setPositionConversionFactor(100);

        m_clampEncoder.setPositionConversionFactor(IntakeConstants.kCamGearRatio);
        m_clampEncoder.setPosition(0.0);

        // SparkMaxPIDController pidController = m_clampMotor.getPIDController();

        // pidController.setP(0.01);
        // pidController.setI(0.0);
        // pidController.setD(0.0);

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void intakeOn(){
        m_intakeMotor.set(IntakeConstants.kCamIntakeSpeed);
        isReversed = false;
    }

    public void intakeOff(){
        this.m_intakeMotor.set(0);
    }

    public void intakeOut(){
        this.m_intakeMotor.set(-IntakeConstants.kCamIntakeSpeed);
    }

    public void intakeReverse(){
        this.m_intakeMotor.set(-IntakeConstants.kCamIntakeSpeed);
        isReversed = true;
    }

    public boolean getIsReversed(){
        return isReversed;
    }

    // public boolean sensorDetected(){
    // return !m_dIOSensor.get();
    // }

    public boolean getHasPiece(){
        return hasPiece;
    }

    public void setHasPiece(boolean hasPiece){
        this.hasPiece = hasPiece;
    }

    public double getRollerSpeed(){
        return(this.m_intakeEncoder.getVelocity());
    }

    public void resetRollerPosition(){
        m_intakeEncoder.setPosition(0);
    }

    public double getRollerPosition(){
        return m_intakeEncoder.getPosition();
    }

    public void sendToDashboard(){
        SmartDashboard.putNumber("intake speed", getRollerSpeed());
        SmartDashboard.putNumber("intake Position", getRollerPosition());
    }

}
