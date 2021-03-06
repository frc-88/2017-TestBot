package org.usfirst.frc.team88.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int driveLeft = 1;
	public static final int driveLeftFollower = 3;
	public static final int driveLeftFollower2 = 5;
	
	public static final int driveRight = 0;
	public static final int driveRightFollower = 2;
	public static final int driveRightFollower2 = 4;
	
	public static final int shifterSolenoidLow = 0;
	public static final int shifterSolenoidHigh = 1;
	
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}

