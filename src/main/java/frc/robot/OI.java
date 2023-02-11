package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.elevator.ElevatorToNode;
import frc.robot.commands.rollerIntake.RollerIntakeOn;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.RollerIntake;
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

	public static void configureButtonBindings(Drivetrain m_robotDrive,Elevator m_elevator,RollerIntake m_intake) {

		//DRIVER//
		// Drive at half speed when the right bumper is held

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
				
		
		// new JoystickButton(operatorController, Button.kLeftBumper.value)
		// 		.onTrue(new ElevatorToNode(m_elevator, Elevator.C));
		
		new JoystickButton(operatorController, Button.kRightBumper.value)
				.toggleOnTrue(new RollerIntakeOn(m_intake));
	
		new JoystickButton(operatorController, Button.kB.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.G));
				
		new JoystickButton(operatorController, Button.kA.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.A));
				
		new JoystickButton(operatorController, Button.kX.value)
				.onTrue(new ElevatorToNode(m_elevator, Elevator.J));

		new JoystickButton(operatorController, Button.kY.value)
		 		.onTrue(new ElevatorToNode(m_elevator, Elevator.C));

		// new JoystickButton(operatorController, Button.kRightStick.value)
		// 		.whenPressed(new /*Command*/);
				
		// new JoystickButton(operatorController, Button.kLeftStick.value)
		// 		.whenPressed(new /*Command*/);	
				
		// new JoystickButton(operatorController, Button.kBack.value)
		// 		.whileHeld(new /*Command*/);
				
		// new JoystickButton(operatorController, Button.kStart.value)
		// 		.whenPressed(new /*Command*/);


		//Climber//

		// new JoystickButton(climberController, Button.kY.value)
		// 	.whenPressed(new /*Command*/);

		// new JoystickButton(climberController, Button.kA.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberController, Button.kStart.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberController, Button.kBack.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberController, Button.kB.value)
		// 	.whenPressed(new /*Command*/);	
		
		// new JoystickButton(climberController, Button.kX.value)
		// 	.whenPressed(new /*Command*/);	
		
		// new JoystickButton(climberController, Button.kRightBumper.value)
		// 	.whenPressed(new /*Command*/);
		
		//Adaptive Controller

		// new JoystickButton(adaptiveController, Button.kStart.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(adaptiveController, Button.kBack.value)
		// 	.whenPressed(new /*Command*/);
			
		// new JoystickButton(adaptiveController, Button.kB.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(adaptiveController, Button.kA.value)
		// 	.whenPressed(new /*Command*/);
	}
}