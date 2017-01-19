package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveFieldOrientated extends Command {
	private static final int DRIVING = 1;
	private static final int PREP = 2;
	private static final int SHIFT = 3;
	private static final double SHIFTSPEED = 500.0;

	private int state;
	private int lastShift;

	public DriveFieldOrientated() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drive.setClosedLoopSpeed();
		// Robot.drive.setOpenLoop();
		Robot.drive.enableRampRate();
		state = DRIVING;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double x, y, speed, magnitude, angle;

		switch (state) {
		case DRIVING:
			magnitude = Robot.oi.getDriverLeftY();

			x = Robot.oi.getDriverRightX();
			y = Robot.oi.getDriverRightY();

			if (x == 0 && y == 0) {
				angle = 0;
			} else {
				angle = Math.toDegrees(Math.atan2(x, y));
			}

			SmartDashboard.putNumber("Joystick Angle", angle);
			SmartDashboard.putNumber("Curve", Robot.drive.getYaw() - angle);
			
			//Robot.drive.driveCurve(magnitude, Robot.drive.getYaw() - angle);

			speed = Math.abs(Robot.drive.getAvgSpeed());
			lastShift++;

			// Comment out in order to use open loop and set the state to
			// permanent drive
			if (Robot.drive.isAutoShift() && (lastShift > (Robot.drive.isLowGear() ? 50 : 5)
					&& ((speed > SHIFTSPEED && Robot.drive.isLowGear() == true)
							|| (speed < SHIFTSPEED && Robot.drive.isLowGear() == false)))) {
				state = PREP;
			}
			break;

		case PREP:
			state = SHIFT;
			break;

		case SHIFT:
			Robot.drive.shift();
			lastShift = 0;
			state = DRIVING;
			break;
		}

		Robot.drive.smartDashboard();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
