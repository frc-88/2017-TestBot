
package org.usfirst.frc.team88.robot.subsystems;

import org.usfirst.frc.team88.robot.Robot;
import org.usfirst.frc.team88.robot.RobotMap;
import org.usfirst.frc.team88.robot.commands.DriveArcade;
import org.usfirst.frc.team88.robot.commands.DriveTank;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drive extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final CANTalon lTalon, lTalonSlave, lTalonSlave2, rTalon, rTalonSlave, rTalonSlave2;
	private final RobotDrive robotDrive;
	private DoubleSolenoid shifter;

	public Drive() {
		lTalon = new CANTalon(RobotMap.driveLeft);
		lTalonSlave = new CANTalon(RobotMap.driveLeftSlave);
		lTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonSlave.set(lTalon.getDeviceID());
		lTalonSlave2 = new CANTalon(RobotMap.driveLeftSlave2);
		lTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonSlave2.set(lTalon.getDeviceID());
		
		rTalon = new CANTalon(RobotMap.driveRight);
		rTalonSlave = new CANTalon(RobotMap.driveRightSlave);
		rTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonSlave.set(rTalon.getDeviceID());
		rTalonSlave2 = new CANTalon(RobotMap.driveRightSlave2);
		rTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonSlave2.set(rTalon.getDeviceID());
		
		shifter = new DoubleSolenoid(RobotMap.shifterSolenoidLow, RobotMap.shifterSolenoidHigh);
		shifter.set(Value.kForward);
		
		robotDrive = new RobotDrive(lTalon, rTalon);
	}

	public void tankDrive(double left, double right)  {
		robotDrive.tankDrive(left, right);
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
	
	public double getLeftSpeed(){
		return lTalon.getSpeed();
	}
	
	public double getRightSpeed(){
		return rTalon.getSpeed();
	}
	
	public double getAvgSpeed(){
		double speed = (rTalon.getSpeed() + lTalon.getSpeed()) / 2;
		return speed;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveTank());
        // setDefaultCommand(new DriveArcade());
    }
}

