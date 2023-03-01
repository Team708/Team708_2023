package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final int kElevator = 40;
    public static final int kTranslation = 40;
    public static final int kRotation = 25;
  }

  // public static final class GoalConstants{
  //   public static final Translation2d kGoalLocation = new Translation2d(8.23,4.115);
  //   public static final Translation2d kWrongBallGoal = new Translation2d(5.50,4.115);
  //   public static final Translation2d kHangerLocation = new Translation2d(2.00,6.00);
  // }

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

    public static final double kFrontLeftOffset = 180-96.85278568891435;//Encoder Offset in Degrees
    public static final double kFrontRightOffset = 180-122.25731340671564;  //Encoder Offset in Degrees
    public static final double kBackLeftOffset = 0.1776169164905552-180;   //Encoder Offset in Degrees
    public static final double kBackRightOffset = 180-111.97272098411736;  //Encoder Offset in Degrees

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

    //.324 - sideways
    //.414 - longways
    public static final Translation2d kFrontLeftLocation = new Translation2d(0.414,0.324); // +X is forward, +Y is to the right 
    public static final Translation2d kFrontRightLocation = new Translation2d(0.414,-0.324); // +X is forward, +Y is to the right
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.414,0.324); // +X is forward, +Y is to the right
    public static final Translation2d kBackRightLocation = new Translation2d(-0.414,-0.324); // +X is forward, +Y is to the right
    // public static final Translation2d kFrontLeftLocation = new Translation2d(0.33,-0.264); // +X is forward, +Y is to the right 
    // public static final Translation2d kFrontRightLocation = new Translation2d(0.33,0.264); // +X is forward, +Y is to the right
    // public static final Translation2d kBackLeftLocation = new Translation2d(-0.376,-0.264); // +X is forward, +Y is to the right
    // public static final Translation2d kBackRightLocation = new Translation2d(-0.376,0.264); // +X is forward, +Y is to the right
     
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

    public static final double kLineupSpeed = 0.3;
    public static final double kLineupAccuracy = 2.0;
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
    public static final int kIntakeMotorID = 41;
    public static final int kClampMotorID = 40;
    public static final int kIntakeEncoderCPR = 42;

    public static final int kRollerGearRatio = 3; // 54 / 18
    public static final double kRollerIntakeSpeed = 1.0;
    
    public static final double kCamGearRatio = 47915 / 486; // 12/74, 18/74, 18/70
    public static final double kCamOpenPose = 2265.0;
    public static final double kCamClosedPose = 0.0;
    public static final double kCamIntakeSpeed = 0.5; //.5
    
    public static final int kIntakeMode = 1; //0 = Roller, 1 = Clamp
  }
  
  /**
   * Static method containing all Elevator constants 
   */
  public static final class ElevatorConstants {
    public static final int kMotorAID = 31;
    public static final int kMotorBID = 32;
    
    // Elevator trig constants
    public static final double kElevatorCosAngle = Math.cos(Math.toRadians(ElevatorConstants.kElevatorAngle));
    public static final double kElevatorSinAngle = Math.sin(Math.toRadians(ElevatorConstants.kElevatorAngle));
    public static final double kElevatorTanAngle = Math.tan(Math.toRadians(ElevatorConstants.kElevatorAngle));

    //x and z in meters
    //Reference point is from front of Robot frame and ground
    //GRABBER
    // public static final Translation2d kGroundPickupPose = new Translation2d(0.5236,-0.031);
    // public static final Translation2d kStartPose = new Translation2d(0.0,0.2); 
    // public static final Translation2d kLowCubePose = new Translation2d(0.7204,0.7989); 
    // public static final Translation2d kLowConePose = new Translation2d(0.8222,1.1065); 
    // public static final Translation2d kHighCubePose = new Translation2d(1.2129,1.1570); 
    // public static final Translation2d kHighConePose = new Translation2d(1.3258,1.3221);
    
    // public static final Translation2d kGroundSafePose = new Translation2d(0.4128,0.3086);
    // public static final Translation2d kLowSafePose = new Translation2d(0.6128,0.7989);
    // public static final Translation2d kMidSafePose = new Translation2d(0.6806,0.9637);
    // public static final Translation2d kHighSafePose = new Translation2d(0.9984,1.3976);

    //ROLLER
    public static final Translation2d kGroundPickupPose = new Translation2d(0.5236,-0.0897);
    public static final Translation2d kStartPose = new Translation2d(0.0,0.2);
    public static final Translation2d kLowCubePose = new Translation2d(0.7795,0.6799);
    public static final Translation2d kLowConePose = new Translation2d(0.8073,0.7399);
    public static final Translation2d kHighCubePose = new Translation2d(1.2161,1.0919);
    public static final Translation2d kHighConePose = new Translation2d(1.2583,1.1241);   //1.4583,1.1141)
    
    public static final Translation2d kGroundSafePose = new Translation2d(0.4528,0.3086);
    public static final Translation2d kLowSafePose = new Translation2d(0.4218,0.7359);
    public static final Translation2d kMidSafePose = new Translation2d(0.600,1.05);
    public static final Translation2d kHighSafePose = new Translation2d(0.9884,1.1576);

    //Elevator Boundaries, meters
    public static final double kLeftBound = -0.051; 
    public static final double kRightBound = 1.16; 
    public static final double kLowerBound = 0;
    public static final double kUpperBound = 1.497;
    public static final double kBumperCoord2 = 0.167925;
    public static final double kBumperCoord1 = 0.25;
    public static final double kMiddleBound = 0.444;
    public static final double kLowConeLeftBound = 0.638;
    public static final double kLowConeUpperBound = 0.878;
    public static final double kCubeMiddleShelf = 0.607;
    public static final double kCubeMiddleShelfBack = 0.877;
    public static final double kCubeTopShelf = 0.907;
    public static final double kHighConeLeftBound = 1.065;
    public static final double kHighConeUpperBound = 1.181;
    public static final double diagy1 = .2;

    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kElevatorAngle = 55;
    public static final double []kPID_A = {20,0,0.0}; //50,0,0
    public static final double []kPID_B = {20,0,0.0}; //{8,0.0,0};
    public static final double kElevatorPulleyRadiusA = Units.inchesToMeters(0.6365);
    public static final double kElevatorPulleyRadiusB = Units.inchesToMeters(0.75);
    public static final double kElevatorGearingA = 44/9; //40:12, 44:30
    public static final double kElevatorGearingB = 20/3; //40:12, 40:20
    public static final double kMinElevatorHeight = 0.0; //m
    public static final double kMaxElevatorHeight = 1.5; //m
    public static final double kMinElevatorReach = -0.2; //m
    public static final double kMaxElevatorReach = 1.219; //m
    
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderConversionFactorA =
        (2.0 * Math.PI * kElevatorPulleyRadiusA) / kElevatorGearingA;

    public static final double kElevatorEncoderConversionFactorB =
        (2.0 * Math.PI * kElevatorPulleyRadiusB) / kElevatorGearingB;
     
    public static final double kRobotBumperThickness = Units.inchesToMeters(3);//m
    public static final double kEndEffectorLength = Units.inchesToMeters(0.01); //22.585);//m
    public static final double kElevatorSetbackFromOrigin = 0.797; //m
    public static final double kElevatorXLength = 0.574;//m
    public static final double kElevatorHeightFromOrigin = 0.215;

    //public static final int kLowSensor = 0;
    //public static final int kHighSensor = 11;
    public static final double kPositionTolerance = 0.1; //0.001; //m

    public static final double kTrajConfigMaxVelocityMPS = 1;
    public static final double kTrajConfigMaxAccelMPSS = 2;
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

    /**
    * Static method containing all robot simulation constants 
    */
    public static final class SimConstants {
      //Game Specific
      //Grid
      public static final double kGridDepth = 1.38; //m
      public static final double kGridHeight = 1.17; //m
      public static final double kBarrierHeight = 0.13; //m
      public static final double kGridRampStart = 0.41; //m
      public static final double kMidConeNodeDepth = 0.58; //m
      public static final double kMidConeNodeHeight = 0.87; //m
      public static final double kHighConeNodeDepth = 1.01; //m
      public static final double kHighConeNodeHeight = 1.17; //m
      public static final double kConeNodeDiameter = 0.04; //m
      public static final double kMidCubeNodeDepth = 0.36; //m
      public static final double kMidCubeNodeHeight = 0.90; //m
      public static final double kHighCubeNodeDepth = 0.80; //m
      public static final double kHighCubeNodeHeight = 0.60; //m
      //Charge Station
      public static final double kChargeStationWidth = 1.93; //m
      public static final double kChargeStationPlatformWidth = 1.22; //m
      public static final double kChargeStationHeight = 0.23; //m
      public static final double kCommunitWidth = 1.54; //m


    }
  }
  