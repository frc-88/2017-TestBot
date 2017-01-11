package org.usfirst.frc.team88.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 */
public class OINetTable extends Subsystem {
    
	public NetworkTable table;

	public OINetTable() {
		table = NetworkTable.getTable("imfeelinglucky");
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

