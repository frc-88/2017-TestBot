package org.usfirst.frc.team88.robot.commands;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoHallway extends CommandGroup {

    public AutoHallway() {
    	addSequential(new DriveZeroYaw(5.0));
    	addSequential(new DriveDistance(31.4));
    	addSequential(new DriveRotateToAngle(-89));
    	addSequential(new DriveDistance(30));
    }
}
