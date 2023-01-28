package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.LockWheels;
// import frc.robot.commands.drive.ResetGyroCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

	// public static double getOperatorRightX() {
	// 	return deadBand(operatorController.getRightX(), ControllerConstants.kOperatorDeadBandRightX);
	// }

	// public static double getOperatorLeftY() {
	// 	return deadBand(operatorController.getLeftY(), ControllerConstants.kOperatorDeadBandLeftY);
	// }

	// public static double getOperatorRightY() {
	// 	return deadBand(operatorController.getRightY(), ControllerConstants.kOperatorDeadBandRightY);
	// }

	// public static double getClimberLeftY() {
	// 	return deadBand(climberController.getLeftY(), ControllerConstants.kClimberDeadBandLeftY);

	// }

	// public static double getClimberRightY() {
	// 	return deadBand(climberController.getRightY(), ControllerConstants.kClimberDeadBandRightY);
	// }

	public static void configureButtonBindings(Drivetrain m_robotDrive) {

		//DRIVER//
		// Drive at half speed when the right bumper is held

		new JoystickButton(driverController, Button.kA.value)
				.whileTrue(new LockWheels(m_robotDrive));

		new JoystickButton(driverController, Button.kB.value)
				.onTrue(new AutoBalance(m_robotDrive));

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
		// 		.whileHeld(new /*Command*/)
		// 		.whenReleased(new /*Command*/);
				
		// new JoystickButton(driverController, Button.kStart.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(driverController, Button.kRightStick.value)
		// 		.whenPressed(new /*Command*/);
				
		// new JoystickButton(driverController, Button.kLeftStick.value)
		// 		.whenPressed(new /*Command*/);
			
		// new DPadButton(driverController, DPadButton.Direction.UP)
		// 		.whenPressed(new /*Command*/);

		// new DPadButton(driverController, DPadButton.Direction.DOWN)
		// 		.whileHeld(new /*Command*/);


		//OPERATOR//
				
		
		// new JoystickButton(operatorController, Button.kLeftBumper.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(operatorController, Button.kRightBumper.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(operatorController, Button.kB.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenPressed(new /*Command*/);

		// new JoystickButton(operatorController, Button.kA.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenReleased(new /*Command*/);

		// new JoystickButton(operatorController, Button.kX.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenReleased(new /*Command*/);

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