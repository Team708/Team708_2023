package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Utilities.JoystickLeftTrigger;
import frc.robot.Utilities.JoystickRightTrigger;
import frc.robot.commands.RequestCone;
import frc.robot.commands.RequestCube;
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
import frc.robot.commands.groups.ClampAndStopWheels;
import frc.robot.commands.groups.OpenAndRunWheels;
import frc.robot.commands.groups.RaiseElevWhenCubeTele;
import frc.robot.commands.groups.RaiseElevWhenPiece;
import frc.robot.commands.groups.RaiseElevWhenPieceTele;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.vision.ActivateAprilTag;
import frc.robot.commands.vision.ActivateLED;
// import frc.robot.commands.vision.CANdleToOrange;
// import frc.robot.commands.vision.CANdleToPurple;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.sim.ElevatorSimulation;
import frc.robot.subsystems.vision.CANdleSystem;
import frc.robot.util.AxisDown;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.Intake;

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

	public static void configureButtonBindings(Drivetrain m_robotDrive, Elevator m_elevator, Intake m_intake, CANdleSystem m_candleSystem) {

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
				
		new JoystickLeftTrigger(driverController)
				.onTrue(new RequestCone(m_candleSystem)); 

		new JoystickLeftTrigger(driverController)
				.and(new JoystickRightTrigger(driverController))
				.onTrue(new RequestCube(m_candleSystem)); 

		new JoystickButton(driverController, Button.kBack.value)
				.onTrue(new ResetDrive(m_robotDrive, new Rotation2d()))
				.onFalse(new SetRumble());

		new JoystickButton(driverController, Button.kStart.value)
				.onTrue(new AutoBalance(m_robotDrive));

				//TODO
		// new JoystickLeftTrigger(driverController)
		// 		.whileTrue(new CANdleToOrange());

		// new JoystickLeftTrigger(driverController)
		// 		.and(new JoystickRightTrigger(driverController))
		// 		.whileTrue(new CANdleToPurple());
					
		new JoystickButton(operatorController, Button.kLeftBumper.value)
		.onTrue(new InstantCommand(() -> m_intake.intakeOff(), m_intake));
		
		// new JoystickButton(operatorController, Button.kRightBumper.value)
		// .onTrue(new OpenAndRunWheels(m_intake));
		
		new JoystickButton(operatorController, Button.kStart.value)
		.onTrue(new IntakeOn(m_intake));

		// new JoystickButton(operatorController, Button.kBack.value)
		// .whileTrue(new GrabberIntakeOut(m_intake));
		// // .onFalse(new GrabberIntakeOff(m_intake));

		new JoystickButton(operatorController, Button.kBack.value)
		.onTrue(new InstantCommand(() -> m_intake.intakeReverse(), m_intake));

		// new JoystickButton(operatorController, Button.kStart.value)
		// .whileTrue(new GrabberIntakeIncClamp(m_intake));
		
		new JoystickButton(operatorController, Button.kB.value)
		.onTrue(new ElevatorToNode(m_elevator, Elevator.F)); //Low/Mid Cone	

		new JoystickButton(operatorController, Button.kB.value)
		.and(new JoystickRightTrigger(operatorController))
		.onTrue(new ElevatorToNode(m_elevator, Elevator.G)); //Low/Mid Cube
		
		new JoystickButton(operatorController, Button.kA.value)
		.onTrue(new RaiseElevWhenPieceTele(m_intake, m_elevator)); //Cone Intake
		
		new JoystickButton(operatorController, Button.kX.value)
		.onTrue(new ElevatorToNode(m_elevator, Elevator.B)); // Ground Safe
		
		new JoystickButton(operatorController, Button.kY.value)
		.onTrue(new ElevatorToNode(m_elevator, Elevator.C)); //High Cone
		
		new JoystickButton(operatorController, Button.kY.value)
		.and(new JoystickRightTrigger(operatorController))
		.onTrue(new ElevatorToNode(m_elevator, Elevator.D)); //High Cube

		new JoystickButton(operatorController, Button.kA.value)
		.and(new JoystickRightTrigger(operatorController))
		.onTrue(new RaiseElevWhenCubeTele(m_intake, m_elevator)); // Cube Intake

		
	}
}