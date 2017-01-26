package org.usfirst.frc.team88.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto40Ball extends CommandGroup {

    public Auto40Ball() {
    	addSequential(new DriveDistance(4));
//    	addSequential(new DriveTurnRight90());
//    	addSequential(new DriveDistance(1.7));
    	addSequential(new DriveDistanceArc(4.2));
    	addSequential(new Delay(0.5));
    	addSequential(new DriveDistanceReverse(-1));
    	addSequential(new DriveTurnRight90());
    }
}
