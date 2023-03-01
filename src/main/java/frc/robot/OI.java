package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Utilities.JoystickLeftTrigger;
import frc.robot.Utilities.JoystickRightTrigger;
import frc.robot.commands.Autonomizations.AutoAlign;
import frc.robot.commands.Autonomizations.AutoBalance;
// import frc.robot.commands.rollerIntake.ToggleRollerForwardReverse;
// import frc.robot.commands.rollerIntake.RollerIntakeOff;
// import frc.robot.commands.rollerIntake.RollerIntakeOn;
// import frc.robot.commands.rollerIntake.IntakeOut;
// import frc.robot.commands.rollerIntake.RollerIntakeReverse;
// import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.ResetDrive;
import frc.robot.commands.drive.SetRumble;
import frc.robot.commands.drive.ToggleFieldOrient;

import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.subsystems.intake.GrabberIntake;
import frc.robot.commands.grabberIntake.GrabberIntakeIncClamp;
import frc.robot.commands.grabberIntake.GrabberIntakeOff;
import frc.robot.commands.grabberIntake.GrabberIntakeOn;
import frc.robot.commands.grabberIntake.GrabberIntakeOpen;
import frc.robot.commands.grabberIntake.GrabberIntakeOut;
import frc.robot.commands.grabberIntake.GrabberIntakeRetraction;
import frc.robot.commands.grabberIntake.ToggleGrabberForwardReverse;
import frc.robot.commands.grabberIntake.ToggleGrabberOpenClosed;
import frc.robot.commands.groups.ClampAndStopWheels;
import frc.robot.commands.groups.OpenAndRunWheels;
import frc.robot.commands.rollerIntake.RollerIntakeOn;
import frc.robot.commands.vision.ActivateAprilTag;
import frc.robot.commands.vision.ActivateLED;
import frc.robot.commands.vision.CANdleToOrange;
import frc.robot.commands.vision.CANdleToPurple;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.RollerIntake;
import frc.robot.subsystems.sim.ElevatorSimulation;
import frc.robot.util.AxisDown;
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

	public static void configureButtonBindings(Drivetrain m_robotDrive, Elevator m_elevator, GrabberIntake m_intake) {

		//DRIVER//
		// Drive at half speed when the right bumper is held

		new JoystickButton(driverController, Button.kLeftBumper.value)
				.onTrue(new ActivateLED())
				.onTrue(new AutoAlign(m_robotDrive));

		new JoystickButton(driverController, Button.kLeftBumper.value)
				.and(new JoystickRightTrigger(driverController))
				.onTrue(new ActivateAprilTag())
				.onTrue(new AutoAlign(m_robotDrive));	

		new JoystickButton(driverController, Button.kLeftStick.value)
				.whileTrue(new ToggleFieldOrient(m_robotDrive)).onFalse(new ToggleFieldOrient(m_robotDrive));

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

		new JoystickButton(driverController, Button.kBack.value)
				.onTrue(new ResetDrive(m_robotDrive, new Rotation2d()))
				.onFalse(new SetRumble());

		new JoystickButton(driverController, Button.kStart.value)
				.onTrue(new AutoBalance(m_robotDrive));

		new JoystickLeftTrigger(driverController)
				.whileTrue(new CANdleToOrange());

		new JoystickLeftTrigger(driverController)
				.and(new JoystickRightTrigger(driverController))
				.whileTrue(new CANdleToPurple());

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
					.onTrue(new InstantCommand(() -> m_intake.intakeOff()));
					
					new JoystickButton(operatorController, Button.kRightBumper.value)
					.onTrue(new OpenAndRunWheels(m_intake));
					
					new JoystickButton(operatorController, Button.kBack.value)
					.onTrue(new InstantCommand(() -> m_intake.intakeReverse()));

					// new JoystickButton(operatorController, Button.kBack.value)
					// .whileTrue(new GrabberIntakeOut(m_intake));
					// // .onFalse(new GrabberIntakeOff(m_intake));

					new JoystickButton(operatorController, Button.kStart.value)
					.onTrue(new InstantCommand(() -> m_intake.intakeOn()));

					// new JoystickButton(operatorController, Button.kStart.value)
					// .whileTrue(new GrabberIntakeIncClamp(m_intake));
					
					new JoystickButton(operatorController, Button.kB.value)
					.onTrue(new ElevatorToNode(m_elevator, Elevator.F)); //Low/Mid Cone	

					new JoystickButton(operatorController, Button.kB.value)
					.and(new JoystickRightTrigger(operatorController))
					.onTrue(new ElevatorToNode(m_elevator, Elevator.G)); //Low/Mid Cube
					
					new JoystickButton(operatorController, Button.kA.value)
					.onTrue(new ElevatorToNode(m_elevator, Elevator.A));   //Ground Pickup
					
					new JoystickButton(operatorController, Button.kX.value)
					.onTrue(new ElevatorToNode(m_elevator, Elevator.B)); // Ground Safe
					
					new JoystickButton(operatorController, Button.kY.value)
					.onTrue(new ElevatorToNode(m_elevator, Elevator.C)); //High Cone
					
					new JoystickButton(operatorController, Button.kY.value)
					.and(new JoystickRightTrigger(operatorController))
					.onTrue(new ElevatorToNode(m_elevator, Elevator.D)); //High Cube

					//operatorController.getRawAxis(0)
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