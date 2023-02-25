package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.rollerIntake.ToggleRollerForwardReverse;
import frc.robot.commands.rollerIntake.RollerIntakeOff;
import frc.robot.commands.rollerIntake.RollerIntakeOn;
import frc.robot.commands.DriveByController;
import frc.robot.commands.drive.InvertDriveCommand;
// import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.ResetDriveCommand;
import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.grabberIntake.GrabberIntakeOff;
import frc.robot.commands.grabberIntake.GrabberIntakeOn;
import frc.robot.commands.grabberIntake.GrabberIntakeOpen;
import frc.robot.commands.grabberIntake.GrabberIntakeRetraction;
import frc.robot.commands.grabberIntake.ToggleGrabberForwardReverse;
import frc.robot.commands.grabberIntake.ToggleGrabberOpenClosed;
import frc.robot.commands.groups.ClampAndStopWheels;
import frc.robot.commands.groups.OpenAndRunWheels;
import frc.robot.commands.drive.ResetDriveCommand;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.GrabberIntake;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.sim.ElevatorSimulation;
import frc.robot.subsystems.drive.Drivetrain;

public class OI {

	// Controllers
	public final static XboxController driverController = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
	public final static XboxController operatorController = new XboxController(ControllerConstants.kOperatorControllerPort); // Operator
	// public final static XboxController climberController  = new XboxController(ControllerConstants.kClimberControllerPort); // Climber
	// public final static XboxController adaptiveController = new XboxController(ControllerConstants.kAdaptiveControllerPort); // Adaptive

	public OI() {

	}

	private static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double getDriverLeftX() {
		return driverController.getLeftX();
	}

	public static double getDriverRightX() {
		return driverController.getRightX();
	}

	public static double getDriverLeftY() {
		return driverController.getLeftY();
	}

	public static double getDriverRightY() {
		return driverController.getRightY();
	}

	// public static double getOperatorLeftX() {
	// 	return deadBand(operatorController.getLeftX(), ControllerConstants.kOperatorDeadBandLeftX);
	// }

	public static double getOperatorRightX() {
		return operatorController.getRightX();
	}

	public static double getOperatorLeftY() {
		return operatorController.getLeftY();
	}

	public static double getOperatorRightY() {
		return operatorController.getRightY();
	}

	// public static double getClimberLeftY() {
	// 	return deadBand(climberController.getLeftY(), ControllerConstants.kClimberDeadBandLeftY);

	// }

	// public static double getClimberRightY() {
	// 	return deadBand(climberController.getRightY(), ControllerConstants.kClimberDeadBandRightY);
	// }

	public static void configureButtonBindings(Drivetrain m_robotDrive,Elevator m_elevator,
	/*RollerIntake m_intake,*/ GrabberIntake m_intake) {

		//DRIVER//
		// Drive at half speed when the right bumper is held

		// new JoystickButton(driverController, Button.kLeftStick.value)
		// 		.whileTrue(new Aim(m_robotDrive));

		new JoystickButton(driverController, Button.kRightStick.value)
				.whileTrue(new LockWheels(m_robotDrive));
				
		// new JoystickButton(driverController, Button.kA.value)
		// 		.onTrue(new TurnToCommand(180, m_robotDrive));
								
		// new JoystickButton(driverController, Button.kB.value)
		// 		.onTrue(new TurnToCommand(90, m_robotDrive));
				
		// new JoystickButton(driverController, Button.kX.value)
		// 		.onTrue(new TurnToCommand(270, m_robotDrive));
				
		// new JoystickButton(driverController, Button.kY.value)
		// 		.onTrue(new TurnToCommand(0, m_robotDrive));

		new JoystickButton(driverController, Button.kA.value)
		.whileTrue(new InstantCommand(() -> DriveByController.getInstance(m_robotDrive).setAutoRotate(180)));

		new JoystickButton(driverController, Button.kB.value)
				.onTrue(new InstantCommand(() -> DriveByController.getInstance(m_robotDrive).setAutoRotate(270)));

		new JoystickButton(driverController, Button.kX.value)
				.onTrue(new InstantCommand(() -> DriveByController.getInstance(m_robotDrive).setAutoRotate(90)));

		new JoystickButton(driverController, Button.kY.value)
				.whileTrue(new InstantCommand(() -> DriveByController.getInstance(m_robotDrive).setAutoRotate(0)));

		new JoystickButton(driverController, Button.kStart.value)
				.onTrue(new InvertDriveCommand(m_robotDrive));

		new JoystickButton(driverController, Button.kBack.value)
				.onTrue(new ResetDriveCommand(m_robotDrive, new Rotation2d()));
				
				// new JoystickButton(driverController, Button.kRightBumper.value)
				// 		.whenPressed(() -> /*Command*/)
				// 		.whenReleased(() -> /*Command*/);
				
				// new JoystickButton(driverController, Button.kLeftBumper.value)
				// 		.whenPressed(() -> /*Command*/)
				// 		.whenReleased(() -> /*Command*/);
				
				// new JoystickButton(driverController, Button.kY.value)
				// 		.whenPressed(new /*Command*/);
				
				// new JoystickButton(driverController, Button.kA.value)
				// 		.whenPressed(new ResetGyroCommand(m_robotDrive));		
				
				// new JoystickButton(driverController, Button.kBack.value)
				// 		.whenPressed(new /*Command*/)
				// 		.whenPressed(new /*Command*/);
				
				// new JoystickButton(driverController, Button.kX.value)
				// 		.whenPressed(new /*Command*/);

		// new JoystickButton(driverController, Button.kB.value)
		// 		.whenPressed(new /*Command*/);
				
		// new JoystickButton(driverController, Button.kStart.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(driverController, Button.kRightStick.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(driverController, Button.kLeftStick.value)
		// 		.whenPressed(new /*Command*/);
		
		// new DPadButton(driverController, DPadButton.Direction.UP)
		// 		.whenPressed(new /*Command*/);
		
		// new DPadButton(driverController, DPadButton.Direction.DOWN)
		// 		.whenPressed(new /*Command*/);


		//OPERATOR//
		
		//_________________________________________________________________
		// new JoystickButton(operatorController, Button.kLeftBumper.value)
		// 		.onTrue(new RollerIntakeOff(m_intake));
		
		// new JoystickButton(operatorController, Button.kRightBumper.value)
		// 		.onTrue(new ToggleRollerForwardReverse(m_intake));

		//MANUAL INTAKE CONTROL
		// new JoystickButton(operatorController, Button.kB.value)
		// 	.whileTrue(new InstantCommand(() -> m_intake.setCamSpeed(0.5), m_intake))
		// 	.onFalse(new ParallelCommandGroup(
		// 		new InstantCommand(() -> m_intake.setCamSpeed(0.0)),
		// 		new InstantCommand(() -> m_intake.setCamPosition(0.0))));
			
		new JoystickButton(operatorController, Button.kLeftBumper.value)
				.onTrue(new ClampAndStopWheels(m_intake));

		new JoystickButton(operatorController, Button.kRightBumper.value)
				.onTrue(new OpenAndRunWheels(m_intake));

		// new JoystickButton(operatorController, Button.kB.value)
		// 		.onTrue(new GrabberIntakeOff(m_intake));

		//__________________________________________________________________
	
		new JoystickButton(operatorController, Button.kB.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.F));
				
		new JoystickButton(operatorController, Button.kA.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.A));
				
		new JoystickButton(operatorController, Button.kX.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.B));

		new JoystickButton(operatorController, Button.kY.value)
		 		.onTrue(new ElevatorToNode(m_elevator, Elevator.C));
		
		// new JoystickButton(operatorController, Button.kLeftStick.value)
		// 				.whileTrue(new (m_robotDrive));

		// new JoystickButton(operatorController, Button.kRightStick.value)
		// 		.whileTrue(new DeployGamePiece(m_robotDrive));

		// new JoystickButton(operatorController, Button.kA.value)
		// 		.onTrue(new AutoBalance(m_robotDrive));
				
		// new JoystickButton(operatorController, Button.kB.value)
		// 		.onTrue(new IntakeOut(m_elevator));
				
		// new JoystickButton(operatorController, Button.kX.value)
		// 		.onTrue(new IntakeIn(m_elevator));
				
		// new JoystickButton(operatorController, Button.kY.value)
		// 		.onTrue(new ()));

		
	}
}