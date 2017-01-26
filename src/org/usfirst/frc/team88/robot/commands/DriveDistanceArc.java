package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveDistanceArc extends Command {
	// states
	private static final int PREP = 0;
	private static final int ACCELERATE = 1;
	private static final int CRUISE = 2;
	private static final int DECELERATE = 3;
	private static final int STOP = 4;
	private static final int END = 5;
	private static final double MAX_SPEED = 1.0;
	private static final double ACCELERATION_SCALE = 0.08;
	private final static double SENSITIVITY = 0.5;
	private final static double CURVE = 0.4;
	
	private int state;
	private double targetDistance;
	private double targetYaw;
	private double rampupDistance;
	private double speed;
	
    public DriveDistanceArc(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	targetDistance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	state = PREP;
    	// should we switch to open loop mode? No, I think.
    	// we should probably at least remove ramp rate
    	Robot.drive.disableRampRate();
    	Robot.drive.resetEncoders();
    	targetYaw = Robot.drive.getYaw();
    	speed = 0.0;
    	
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
    	case PREP:
    		if (Robot.drive.getAvgPosition() < 1) {
    			state = ACCELERATE;
    		}
    		break;
    	case ACCELERATE: // ramping up, gradually increase velocity until we get to desired speed
    		speed = speed + ACCELERATION_SCALE;
    		Robot.drive.driveCurve(speed, CURVE, SENSITIVITY);
    		
    		if (Robot.drive.getAvgPosition() > targetDistance / 2.0) {
    			state = DECELERATE;
    		}
    		
    		if (speed > MAX_SPEED) {
    			rampupDistance = Robot.drive.getAvgPosition();
    			state = CRUISE;
    		}
    		break;
    	case CRUISE: // consistent speed until we get close
    		Robot.drive.driveCurve(speed, CURVE, SENSITIVITY);
    		
    		if (Robot.drive.getAvgPosition() > targetDistance - rampupDistance) {
    			state = DECELERATE;
    		}
    		break;
    	case DECELERATE: // slow down as we approach target, gradually decrease velocity
    		speed = speed - ACCELERATION_SCALE;
    		Robot.drive.driveCurve(speed, CURVE, SENSITIVITY);
    		if (speed < 0.02) {
    			state = STOP;
    		}
    		break;
    	case STOP: //stop
    		Robot.drive.driveCurve(0.0, 0.0);
    		state = END;
    		break;
    	}
    	
    	SmartDashboard.putNumber("Target Yaw", targetYaw);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return state == END;
    }

    // Called once after is	Finished returns true
    protected void end() {
    	// reset and configuration changes made in init
    	// do same when interrupted
    	Robot.drive.enableRampRate();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		Robot.drive.driveCurve(0.0, 0.0);
		Robot.drive.enableRampRate();
    }
}
