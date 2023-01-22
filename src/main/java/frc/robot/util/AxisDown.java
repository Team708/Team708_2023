package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;


/**
 *	Makes moving an axis up on a joystick result in a button input
 */
public class AxisDown extends Trigger {
    
	private XboxController Controller;
	private int axis;
	
	public AxisDown(XboxController targetController, int targetAxis) {
		Controller = targetController;
		axis = targetAxis;
	}
	
    public boolean get() {
        return (Controller.getRawAxis(axis) <= .1);
    }
}
