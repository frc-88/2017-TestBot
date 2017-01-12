package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveFollowTarget extends Command {
	double targetAngle;
	
    public DriveFollowTarget() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
       	Robot.drive.rotateController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (Robot.jetsonTable.getDouble("Distance", -1.0) != -1.0) {
        	targetAngle = Robot.drive.getYaw() + Robot.jetsonTable.getDouble("Angle", 0.0);
        	Robot.drive.rotateController.setSetpoint(targetAngle);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	SmartDashboard.putNumber("Rot Error", Robot.drive.rotateController.getError());
		SmartDashboard.putNumber("Rot On Target", Robot.drive.rotateController.onTarget()?1:0);

		return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.drive.rotateController.disable();    	
   }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		Robot.drive.rotateController.disable();
   }
}
