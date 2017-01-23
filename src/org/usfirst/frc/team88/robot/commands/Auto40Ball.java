package org.usfirst.frc.team88.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto40Ball extends CommandGroup {

    public Auto40Ball() {
    	addSequential(new DriveDistance(7.5));
    	addSequential(new DriveTurnRight90());
    	addSequential(new DriveDistance(3.5));
    }
}
