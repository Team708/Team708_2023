package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
  
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final String modID;
  private final double offset;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  // Create a Potentiometer to store the output of the absolute encoder that
  // tracks the angular position of the swerve module
  //private final AnalogPotentiometer m_turningEncoder;

  // private final CANCoder canCoder;

  boolean driveEncoderReversed;
  boolean turnEncoderReversed;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  
  private final PIDController m_turnPIDController = new PIDController(ModuleConstants.kPModuleTurningController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // private final ProfiledPIDController m_turningPIDCalculator = new ProfiledPIDController(
  //     ModuleConstants.kPModuleTurningController, 0.0, 0.0,
  //     new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  //         ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  // private final PIDController m_turningPIDCalculator = new PIDController(1, 0, 0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(String modID, int driveMotorChannel, int turningMotorChannel, boolean driveEncoderReversed,
      double offset/*, int CANCoderChannel*/) {
    this.modID = modID;
    this.offset = offset;
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushed); //kBrushless

    m_turningMotor.setInverted(turnEncoderReversed);

    m_driveEncoder = m_driveMotor.getEncoder();
    // m_turnEncoder = m_turningMotor.getEncoder();
    m_turnEncoder = m_turningMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, Constants.ModuleConstants.kEncoderCPR);
    //canCoder = new CANCoder(CANCoderChannel);

    this.driveEncoderReversed = driveEncoderReversed;

    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // m_turningPIDCalculator.reset(0.0);
    // m_turnEncoder.setPosition(0);
    
    configureRobotMotors(/*canCoder*/);
    //resetEncoders();

    // Set whether drive encoder should be reversed or not
    m_driveMotor.setInverted(driveEncoderReversed);
  }

  private void configureRobotMotors(/*CANCoder offsetEncoder*/){
    // drivePIDController.setFeedbackDevice(driveEncoder);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityPerPulse * ModuleConstants.kVelocityModifier);
    m_driveEncoder.setPosition(0.0);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    // drivePIDController.setP(0.2);
    // drivePIDController.setI(0);
    // drivePIDController.setD(24);

    m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);    
    m_turnEncoder.setInverted(false);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    // m_turnEncoder.setPosition(offsetEncoder.getAbsolutePosition());
  }

  /**
   * Returns the current angle the module.
   * @return The current angle of the module in radians.
   */
  public double getAngle() {
    return(m_turnEncoder.getPosition() + (offset * Math.PI / 180));
  }

  /**
   * Method to get position of the wheel
   * @return The position of the wheel.
   */
  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SmartDashboard.putNumber(modID + " Desired Position", desiredState.angle.getRadians());//state.angle.getRadians());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));
    SmartDashboard.putNumber(modID + " State", state.angle.getRadians());
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turnPIDController.calculate(getAngle(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    
    m_turningMotor.set(turnOutput);
    SmartDashboard.putNumber(modID + "output", turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    //m_turningMotor.setPosition(0);
  }

  public void invertDrive(){
    if (m_driveMotor.getInverted()){
      m_driveMotor.setInverted(false);
    }
    else {
      m_driveMotor.setInverted(true);
    }
  }

  public void sendToDashboard() {
    // SmartDashboard.putNumber(modID + " drive pos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(modID + " drive vel", m_driveEncoder.getVelocity());
    if(Constants.RobotConstants.kRobot){
      // SmartDashboard.putNumber(modID + " turn pos",
      // ((180/Math.PI)*(m_turningMotorSRX.getSelectedSensorPosition(0) * ModuleConstants.kTurningEncoderDistancePerPulse)
      // + (offset * Math.PI / 180)));
      // SmartDashboard.putNumber(" Angle", getAngle());
    }else{
      //TODO NEO version of output
    }
    // SmartDashboard.putNumber(modID + "state velocity", getState().speedMetersPerSecond);
    // SmartDashboard.putNumber(modID + " turn vel",
    // m_turningMotor.getSelectedSensorVelocity(0) *
    // ModuleConstants.kTurningEncoderDistancePerPulse *180/Math.PI);
    SmartDashboard.putNumber(modID + " turn angle degrees", (getAngle() * (180 / Math.PI))-offset);
    SmartDashboard.putNumber(modID + " Encoder Value", m_turnEncoder.getPosition());
    SmartDashboard.putNumber("ENCODERCPR", m_turnEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber(modID+ "angle", getAngle());
  }
}
