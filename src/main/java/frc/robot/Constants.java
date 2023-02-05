package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utilities.LinearInterpolationTable;

import java.awt.geom.Point2D;

  /**
   * Static method containing all constant values for the robot in one location
   */
public final class Constants {
  /**
   * Static method containing all Global constants 
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6;        //Sets a voltage compensation value ideally 12.6V
    public static final int PCMID = 50;
    public static final int PDPID = 51;
    public static final double kLoopTime = 0.020;
  }

  public static final class CurrentLimit{
    public static final int kIntake = 25;
    public static final int kElevator = 30;
    public static final int kTranslation = 40;
    public static final int kRotation = 25;
  }

  public static final class GoalConstants{
    public static final Translation2d kGoalLocation = new Translation2d(8.23,4.115);
    public static final Translation2d kWrongBallGoal = new Translation2d(5.50,4.115);
    public static final Translation2d kHangerLocation = new Translation2d(2.00,6.00);
  }

  /**
   * Static method containing all Drivetrain constants 
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 25;   //CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 27;  //CANID of the Translation SparkMAX
    public static final int kRearLeftDriveMotorPort = 21;    //CANID of the Translation SparkMAX
    public static final int kRearRightDriveMotorPort = 23;   //CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 26;   //CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 28;  //CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 22;    //CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 24;   //CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 12;   //Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 14;  //Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 13;    //Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 11;   //Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -96.85278568891435;//Encoder Offset in Degrees
    public static final double kFrontRightOffset = -122.25731340671564;  //Encoder Offset in Degrees
    public static final double kBackLeftOffset = 0.1776169164905552;   //Encoder Offset in Degrees
    public static final double kBackRightOffset = -111.97272098411736;  //Encoder Offset in Degrees

    //Drive motor PID is best done on the roboRIO currently as the SparkMAX does not allow for static gain values on the PID controller, 
    //    these are necessary to have high accuracy when moving at extremely low RPMs
    //public static final double[] kFrontLeftTuningVals   =   {0.0120,0.2892,0.25,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kFrontRightTuningVals  =   {0.0092,0.2835,0.25,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackLeftTuningVals    =   {0.0142,0.2901,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackRightTuningVals   =   {0.0108,0.2828,0.25,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals   =   {0.001,0.2850,0.2,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kFrontRightTuningVals  =   {0.001,0.2850,0.2,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackLeftTuningVals    =   {0.001,0.2850,0.2,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackRightTuningVals   =   {0.001,0.2850,0.2,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.33,-0.264); // +X is forward, +Y is to the right
    public static final Translation2d kFrontRightLocation = new Translation2d(0.33,0.264); // +X is forward, +Y is to the right
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.376,-0.264); // +X is forward, +Y is to the right
    public static final Translation2d kBackRightLocation = new Translation2d(-0.376,0.264); // +X is forward, +Y is to the right
     
    //Because the swerve modules poisition does not change, define a constant SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics 
      = new SwerveDriveKinematics(kFrontLeftLocation,kFrontRightLocation,kBackLeftLocation,kBackRightLocation);

    public static final double kMaxAcceleration = 3.75;
    public static final double kMaxSpeedMetersPerSecond = 3.5; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = 4.0;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = 6.25;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kInnerDeadband = 0.08; //This value should exceed the maximum value the analog stick may read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; //This value should be lower than the analog stick X or Y reading when aimed at a 45deg angle (Such that X and Y are are maximized simultaneously)
  
    //Minimum allowable rotation command (in radians/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed * Math.pow(DriveConstants.kInnerDeadband,2);
    //Minimum allowable tranlsation command (in m/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond * Math.pow(DriveConstants.kInnerDeadband,2);

    public static final double[] kKeepAnglePID = { 0.700, 0, 0 }; //Defines the PID values for the keep angle PID

  }
  /**
   * Static method containing all Swerve Module constants 
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0;          //Units of %power/s, ie 4.0 means it takes 0.25s to reach 100% power from 0%
    private static final double kTranslationGearRatio = 5.6111111; //Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0777*0.98;           //Wheel Diameter in meters, may need to be experimentally determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; //Calculates the conversion factor of RPM of the translation motor to m/s at the floor

    //NOTE: You shoulds ALWAYS define a reasonable current limit when using brushless motors 
    //      due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; //Defines the PID values for rotation of the serve modules, should show some minor oscillation when no weight is loaded on the modules
  }
  
  /**
   * Static method containing all Vision/Limelight constants 
   */
  public static final class VisionConstants {
    public static final double kElevationOffset =38.5;              // Degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = -0.50;                // Degree azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0;  // Center Height of the Target in inches above the lens
    public static final double kTrackTolerance = 0.0200;             // Allowable Limelight angle error in radians
  
    public static final int kVisionLedOn = 0;
    public static final int kVisionLedOff = 1;
  }
  
  public static final class CandleConstants {
    public static final int kCANdleID    = 1;
  public static final int kMaxBrightnessAngle  = 90;
  public static final int kMidBrightnessAngle  = 180;
  public static final int kZeroBrightnessAngle = 270;
  }

  /**
   * Static method containing all Intake constants 
   */
  public static final class IntakeConstants {
    
  }
  
  /**
   * Static method containing all Elevator constants 
   */
  public static final class ElevatorConstants {
    //public static final int kMotor1ID = 31;
    //public static final int kMotor2ID = 16;
    //public static final double []kPIDF = {0.000075,0,0,0.000091};
    //public static final int kLowSensor = 0;
    //public static final int kHighSensor = 11;
  }
  
  /**
   * Static method containing all Autonomous constants 
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.5; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    
    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;
    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeed, kMaxAngularAccel); //Creates a trapezoidal motion for the auto rotational commands
    }
    
    /**
     * Static method containing all User Controller constants 
     */
    public static final class ControllerConstants{
      public static final int kDriverControllerPort = 0;    //When making use of multiple controllers for drivers each controller will be on a different port
      public static final int kOperatorControllerPort = 1;  //When making use of multiple controllers for drivers each controller will be on a different port
      
    }
  }
  