package org.usfirst.frc.team88.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team88.robot.commands.PutBoolean;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static final int LEFT_VERT_AXIS = 1;
	private static final int RIGHT_VERT_AXIS = 5;
	private static final double DEADZONE = 0.15;
	
	// driver controller setup
	private Joystick driverController = new Joystick(0);
	private Button driverButtonA = new JoystickButton(driverController, 1);
	private Button driverButtonB = new JoystickButton(driverController, 2);
	private Button driverButtonX = new JoystickButton(driverController, 3);
	private Button driverButtonY = new JoystickButton(driverController, 4);
	private Button driverButtonLeftBumper = new JoystickButton(driverController, 5);
	private Button driverButtonRightBumper = new JoystickButton(driverController, 6);
	private Button driverButtonBack = new JoystickButton(driverController, 7);
	private Button driverButtonStart = new JoystickButton(driverController, 8);
	private Button driverButtonLeftAxisPress = new JoystickButton(driverController, 9);
	private Button driverButtonRightAxisPress = new JoystickButton(driverController, 10);

	public OI() {
		driverButtonA.whenPressed(new PutBoolean("driverButtonA", true));
		driverButtonA.whenReleased(new PutBoolean("driverButtonA", false));

		driverButtonB.whenPressed(new PutBoolean("driverButtonB", true));
		driverButtonB.whenReleased(new PutBoolean("driverButtonB", false));

		driverButtonX.whenPressed(new PutBoolean("driverButtonX", true));
		driverButtonX.whenReleased(new PutBoolean("driverButtonX", false));

		driverButtonY.whenPressed(new PutBoolean("driverButtonY", true));
		driverButtonY.whenReleased(new PutBoolean("driverButtonY", false));
	}
	
	
	public double getDriverRightVerticalAxis() {
		return driverController.getRawAxis(RIGHT_VERT_AXIS);
	}
	
	public double getDriverLeftVerticalAxis() {
		return driverController.getRawAxis(LEFT_VERT_AXIS);
	}



	public double applyDeadZone(double value) {
		if (Math.abs(value) < DEADZONE) {
			return 0.0;
		} else if (value > 0) {
			value = (value - DEADZONE) / (1 - DEADZONE);
		} else {
			value = (value + DEADZONE) / (1 - DEADZONE);
		}

		return value;
	}
	}

