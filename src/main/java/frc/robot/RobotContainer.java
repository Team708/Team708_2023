// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveByController;
import frc.robot.commands.Autos.DriveStraightAuto;
import frc.robot.commands.Autos.SigmoidPathAuto;
import frc.robot.commands.Autos.DriveToPieceAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.drive.Drivetrain;

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

  private final Drivetrain m_drive = new Drivetrain();

  private final DriveByController m_driveByController
    = new DriveByController(m_drive, OI.driverController);

  private final Command doNothin = new WaitCommand(20.0);
  private final Command SigmoidPath = new SigmoidPathAuto(m_drive, 1);
  private final Command DriveStraight = new DriveStraightAuto(m_drive, 1);
  private final Command DriveToPiece = new DriveToPieceAuto(m_drive, 1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoChooser();

    m_drive.setDefaultCommand(m_driveByController);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new POVButton(OI.driverController, 0)
        .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Rotation2d(0.0))));

    OI.configureButtonBindings(m_drive);

  }


  private void configureAutoChooser(){
    m_chooser.addOption("Do Nothing", doNothin);
    m_chooser.addOption("Sigmoid", SigmoidPath);
    m_chooser.addOption("DriveStraight", DriveStraight);
    m_chooser.addOption("DriveToPiece", DriveToPiece);
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

  public void sendToDashboard() {
    m_drive.sendToDashboard();
  }
}
