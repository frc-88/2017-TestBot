package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistance extends Command {
	private int state;
	private double target;
	
    public DriveDistance(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	target = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	state = 1;
    	// should we switch to open loop mode? No, I think.
    	// we should probably at least remove ramp rate
    	
    	// Once we gather data from practical tests, 
    	// we should calculate various state change points
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// throughout this, we want to drive straight, so we should use navX yaw
    	// in order to stay on target. See RobotDrive.drive(magnitude,curve) source code 
    	// see:
    	// https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/athena/java/edu/wpi/first/wpilibj/RobotDrive.java#L149
    	//  
    	
    	switch (state) {
    	case 1: // ramping up, gradually increase velocity until we get to desired speed
    		break;
    	case 2: // consistent speed until we get close
    		break;
    	case 3: // slow down as we approach target, gradually decrease velocity
    		break;
    	case 4: //stop
    		break;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return state == 5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	// reset and configuration changes made in init
    	// do same when interrupted
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
