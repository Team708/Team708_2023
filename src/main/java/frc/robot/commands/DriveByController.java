package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.Utilities.MathUtils;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

  /**
   * Implements a DriveByController command which extends the CommandBase class
   */
public class DriveByController extends CommandBase {
  private final Drivetrain m_robotDrive;

  private static DriveByController instance = null;

  public static DriveByController getInstance(Drivetrain drive){
    if(instance == null){
      instance = new DriveByController(drive);
    }
    return instance;
  }

  double autoAngle = 0.0;
  boolean autoRotEnabled = false;
  double lastSpeed = 0.0;
  double lastTime = Timer.getFPGATimestamp();
  ProfiledPIDController controller = new ProfiledPIDController(0.05, 0.00, 0.004, new Constraints(3000, 1500));

  /**
   * Contructs a DriveByController object which applys the driver inputs from the
   * controller to the swerve drivetrain
   * 
   * @param drive      is the swerve drivetrain object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  private DriveByController(Drivetrain drive) {
    m_robotDrive = drive; // Set the private member to the input drivetrain
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(0.5, 10); //Degrees?
    addRequirements(m_robotDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  @Override
  public void initialize(){
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond;
    double desiredX = -inputTransform(OI.getDriverLeftY())*maxLinear;
    double desiredY = -inputTransform(OI.getDriverLeftX())*maxLinear;
    Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
    double desiredMag = desiredTranslation.getDistance(new Translation2d());
    double desiredRot = -inputTransform(OI.getDriverRightX())* DriveConstants.kMaxAngularSpeed;//desiredRot = 0.0;

    if(Math.abs(desiredRot) > 0.08){
      autoRotEnabled = false;
    }
    if(autoRotEnabled){
        //double currPoseDegrees = MathUtil.inputModulus(m_robotDrive.getPose().getRotation().getDegrees(), 0, 360);
        // double currPoseDegrees = m_robotDrive.getPose().getRotation().getDegrees();
        double currDegrees = MathUtil.inputModulus(m_robotDrive.getGyroDegrees(),-180,180);
        // System.out.println(currDegrees + "       " + autoAngle);
        desiredRot = controller.calculate(currDegrees, autoAngle);
        SmartDashboard.putNumber("currDegrees", currDegrees);
        SmartDashboard.putNumber("autoAngle", autoAngle);
        SmartDashboard.putNumber("RobotAngle", m_robotDrive.getPose().getRotation().getDegrees());
        if(controller.atSetpoint()){
          autoRotEnabled = false;
        }
    }

    // System.out.println(manualRotEnabled);
    
    if(desiredMag >= maxLinear){
      desiredTranslation.times(maxLinear/desiredMag);
    }
    m_robotDrive.drive(desiredTranslation.getX(), 
                       desiredTranslation.getY(),
                       desiredRot,
                       m_robotDrive.getFieldOrient(),
                       true);
  }

  @Override
  public void end(boolean interrupted){

  }

  /**
   * This function takes the user input from the controller analog sticks, applys a deadband and then quadratically
   * transforms the input so that it is easier for the user to drive, this is especially important on high torque motors 
   * such as the NEOs or Falcons as it makes it more intuitive and easier to make small corrections
   * @param input is the input value from the controller axis, should be a value between -1.0 and 1.0
   * @return the transformed input value
   */
  private double inputTransform(double input){
    //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
  }

  public void setAutoRotate(double angle){
    autoAngle = angle;
    autoRotEnabled = true;
  }

}
