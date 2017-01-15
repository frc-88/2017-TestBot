package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveRotateToAngle extends Command {
	double targetAngle;
	
    public DriveRotateToAngle(double angle) {
        requires(Robot.drive);

        targetAngle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.drive.setOpenLoop();
		Robot.drive.rotateController.enable();    	
		Robot.drive.rotateController.setSetpoint(targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		SmartDashboard.putNumber("Rot Error", Robot.drive.rotateController.getError());
		SmartDashboard.putNumber("Rot On Target", Robot.drive.rotateController.onTarget()?1:0);

		return Robot.drive.rotateController.onTarget();}

    // Called once after isFinished returns true
    protected void end() {
		Robot.drive.rotateController.disable();
		Robot.drive.setClosedLoopSpeed();
   }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		Robot.drive.rotateController.disable();    	
		Robot.drive.setClosedLoopSpeed();
   }
}
