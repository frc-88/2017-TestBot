
package org.usfirst.frc.team88.robot.subsystems;

import org.usfirst.frc.team88.robot.Robot;
import org.usfirst.frc.team88.robot.RobotMap;
import org.usfirst.frc.team88.robot.commands.DriveArcade;
import org.usfirst.frc.team88.robot.commands.DriveTank;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drive extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final CANTalon lTalon, lTalonSlave, rTalon, rTalonSlave;
	private final RobotDrive robotDrive;

	public Drive() {
		lTalon = new CANTalon(RobotMap.driveLeft);
		lTalonSlave = new CANTalon(RobotMap.driveLeftSlave);
		lTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonSlave.set(lTalon.getDeviceID());
		
		rTalon = new CANTalon(RobotMap.driveRight);
		rTalonSlave = new CANTalon(RobotMap.driveRightSlave);
		rTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonSlave.set(rTalon.getDeviceID());
		
		robotDrive = new RobotDrive(lTalon, rTalon);
	}

	public void tankDrive(double left, double right)  {
		robotDrive.tankDrive(left, -right);
	}
	
	public void arcadeDrive() {
		robotDrive.arcadeDrive(Robot.oi.driverController);
	}
	
	public double getLeftCurrent(){
		return lTalon.getOutputCurrent();
	}
	
	public double getRightCurrent(){
		return rTalon.getOutputCurrent();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveTank());
        // setDefaultCommand(new DriveArcade());
    }
}

