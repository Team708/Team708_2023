package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.OI;
import frc.robot.Constants.CurrentLimit;

public class GrabberIntake extends SubsystemBase{
    
    private CANSparkMax m_intakeMotor;
    private CANSparkMax m_clampMotor;
    private RelativeEncoder m_clampEncoder;
    private SparkMaxPIDController pidController;
    
    private boolean isOpen = false;
    private boolean isReversed = false;
    private DigitalInput m_dIOSensor;

    public GrabberIntake(DigitalInput m_dIOSensor){

        this.m_dIOSensor = m_dIOSensor;

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

    public void setCamSpeed(double output){
        // m_clampMotor.setVoltage(output);
        m_clampMotor.set(output);
    }

    public void setCamPosition(double position){
        m_clampEncoder.setPosition(position);
    }

    public double getCamPosition(){
        return m_clampEncoder.getPosition();
    }

    // public void setCamPoint(double position){
    //         m_clampEncoder.setPoint(position);
    //     }

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

    public void incClamp(){
            this.m_clampMotor.set(.5);
        }

    public void stopClamp(){
        this.m_clampMotor.set(0);
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

    public boolean sensorDetected(){
    return !m_dIOSensor.get();
    }

    public double getRollerSpeed(){
        return(this.m_intakeMotor.getEncoder().getVelocity());
    }

    public void sendToDashboard(){
        SmartDashboard.putNumber("Intake Encoder Val", getCamPosition());
        SmartDashboard.putBoolean("Sensor", sensorDetected());
        SmartDashboard.putNumber("intake speed", getRollerSpeed());
    }

}
