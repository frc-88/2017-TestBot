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
	private static final double SHIFTSPEED = 500.0;
	
    public DriveTank() {
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.setClosedLoopSpeed();
    	//Robot.drive.setOpenLoop();
    	Robot.drive.enableRampRate();
    	state = DRIVING;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	switch (state){
    	case DRIVING:
        	left = Robot.oi.getDriverLeftVerticalAxis();
        	right = Robot.oi.getDriverRightVerticalAxis();
        	
        	left = Robot.oi.applyDeadZone(left);
        	right = Robot.oi.applyDeadZone(right);
        	
        	speed = Math.abs(Robot.drive.getAvgSpeed());
        	
        	Robot.drive.setTarget(left, right);
        	lastShift++;
        
        	//Comment out in order to use open loop and set the state to permanent drive
        	if (Robot.drive.isAutoShift() && (lastShift > (Robot.drive.isLowGear() ? 50 : 5) && 
        			((speed > SHIFTSPEED && Robot.drive.isLowGear() == true)||
         			 (speed < SHIFTSPEED && Robot.drive.isLowGear() == false)))){
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
