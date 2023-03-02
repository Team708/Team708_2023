// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveByController;
import frc.robot.commands.OperateByController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CANdleSystem;
import frc.robot.commands.Autos.DriveStraightAuto;
import frc.robot.commands.Autos.DriveToPieceAuto;
import frc.robot.commands.Autos.LeftDriveToPieceAuto;
import frc.robot.commands.Autos.LineAndBalanceAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  // private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  private final DigitalInput dIO = new DigitalInput(0);

  private final Drivetrain m_drive = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  // private final RollerIntake m_intake = new RollerIntake(dIO);
  private final Intake m_intake = new Intake(dIO);
  private final CANdleSystem m_candle = new CANdleSystem();

  private final DriveByController m_driveByController
    =  DriveByController.getInstance(m_drive);

  private final OperateByController m_operateByController
    = new OperateByController(m_elevator);

  private final Command doNothin         = new WaitCommand(15);
  private final Command DriveStraight    = new DriveStraightAuto(m_drive,    2);
  private final Command DriveToPiece     = new DriveToPieceAuto(m_drive,     4, m_elevator, m_intake);
  private final Command LeftDriveToPiece = new LeftDriveToPieceAuto(m_drive, 4, m_elevator, m_intake);
  private final Command ScoreLineBalance = new LineAndBalanceAuto(m_drive,   2, m_elevator, m_intake);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoChooser();

    m_drive.setDefaultCommand(m_driveByController);
    m_elevator.setDefaultCommand(m_operateByController);

    m_drive.resetOdometry(new Pose2d()); //added to test JP
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new POVButton(OI.driverController, 0)
    //     .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Rotation2d(0.0))));  //JNP

    OI.configureButtonBindings(m_drive, m_elevator, m_intake, m_candle);
  }


  private void configureAutoChooser(){
    m_chooser.addOption("Leftside  DriveStraight",  DriveStraight);
    m_chooser.addOption("Leftside  DriveToPiece",  LeftDriveToPiece);
    m_chooser.addOption("Rightside DriveToPiece",   DriveToPiece);
    m_chooser.addOption("ScoreLineBalance", ScoreLineBalance);
    
    m_chooser.addOption("Do Nothing",     doNothin);
    m_chooser.setDefaultOption("Do Nothing", doNothin);
    SmartDashboard.putData(m_chooser);  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  public void simulationInit(){
    m_elevator.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  public void simulationPeriodic() {
    m_elevator.simulationPeriodic();
  }

  public void sendToDashboard() {
    m_drive.sendToDashboard();
    m_elevator.sendToDashboard();
    m_intake.sendToDashboard();
  }
}
