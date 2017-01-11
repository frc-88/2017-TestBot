package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DriveShift extends InstantCommand {

    public DriveShift() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    }

    // Called once when the command executes
    protected void initialize() {
    	if (!Robot.drive.isAutoShift()) {
    		Robot.drive.shift();
    	}
    }

}
