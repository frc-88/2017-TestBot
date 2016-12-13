package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTank extends Command {
	private double left;
	private double right;
	private int state;
	private int lastShift;
	private double speed;
	private static final int DRIVING = 1;
	private static final int PREP = 2;
	private static final int SHIFT = 3;
	private static final double SHIFTSPEED = 550.0;

	public DriveTank() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
		requires(Robot.oiNetTable);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drive.setClosedLoopSpeed();
		// Robot.drive.setOpenLoop();
		state = DRIVING;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		switch (state) {
		case DRIVING:
			left = Robot.oi.getDriverLeftVerticalAxis();
			right = Robot.oi.getDriverRightVerticalAxis();

			left = Robot.oi.applyDeadZone(left);
			right = Robot.oi.applyDeadZone(right);

			Robot.oiNetTable.table.putDouble("Right Current", Robot.drive.getRightCurrent());
			Robot.oiNetTable.table.putDouble("Left Current", Robot.drive.getLeftCurrent());

			Robot.drive.smartDashboard(state);

			speed = Robot.drive.getAvgSpeed();

			// Robot.drive.tankDrive(left, right);
			Robot.drive.closedLoopDrive(left, right);
			lastShift++;

			// Comment out in order to use open loop and set the state to
			// permanent drive
			if (Robot.drive.isAutoShift() && (lastShift > 100)
					&& ((speed > SHIFTSPEED && Robot.drive.isLowGear() == true)
							|| (speed < SHIFTSPEED && Robot.drive.isLowGear() == false))) {
				state = PREP;
			}
			break;

		case PREP:
			left = Robot.drive.getLeftSpeed();
			right = Robot.drive.getRightSpeed();

			Robot.drive.smartDashboard(state);

			// Robot.drive.tankDrive(left, right);
			Robot.drive.closedLoopDrive(left, right);
			state = SHIFT;
			break;

		case SHIFT:
			Robot.drive.smartDashboard(state);

			Robot.drive.shift();
			lastShift = 0;
			Robot.oiNetTable.table.putBoolean("Has Shifted", Robot.drive.isLowGear());
			state = DRIVING;
			break;
		}
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
