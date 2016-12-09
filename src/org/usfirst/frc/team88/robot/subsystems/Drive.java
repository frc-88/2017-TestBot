
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final CANTalon lTalon, lTalonSlave, lTalonSlave2, rTalon, rTalonSlave, rTalonSlave2;
	private final RobotDrive robotDrive;
	private DoubleSolenoid shifter;
	
	private final double FAST_SPEED = 400.0;
	private final double SLOW_SPEED = 200.0;
	
	public final static double ENC_CYCLES_PER_REV = 360.0;
	public final static double GEAR_RATIO = (3 * 34) / 50;
	public final static double WHEEL_DIAMETER = 4;
	
	private final static int SPEED_PROFILE = 0;
	private final static double SPEED_RAMPRATE = 1;
	private final static double SPEED_P = 0;
	private final static double SPEED_I = 0;
	private final static double SPEED_D = 0;
	private final static double SPEED_F = 1;
	private final static int SPEED_IZONE = 0;
	
	private final static int POSITION_PROFILE = 1;
	private final static double POSITION_RAMPRATE = 0.1;
	private final static double POSITION_P = 0.8;
	private final static double POSITION_I = 0.0;
	private final static double POSITION_D = 0.0;
	private final static double POSITION_F = 0.0;
	private final static int POSITION_IZONE = 0;
	
	private double maxSpeed;
	private CANTalon.TalonControlMode controlMode;

	public Drive() {
		lTalon = new CANTalon(RobotMap.driveLeft);
		lTalon.setPID(SPEED_P, SPEED_I, SPEED_D, SPEED_F, SPEED_IZONE, SPEED_RAMPRATE, SPEED_PROFILE);
		lTalon.setPID(POSITION_P, POSITION_I, POSITION_D, POSITION_F, POSITION_IZONE, POSITION_RAMPRATE, POSITION_PROFILE);
		lTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lTalon.configEncoderCodesPerRev(360);
		
		lTalonSlave = new CANTalon(RobotMap.driveLeftSlave);
		lTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonSlave.set(lTalon.getDeviceID());
		
		lTalonSlave2 = new CANTalon(RobotMap.driveLeftSlave2);
		lTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonSlave2.set(lTalon.getDeviceID());
		
		rTalon = new CANTalon(RobotMap.driveRight);
		rTalon.setPID(SPEED_P, SPEED_I, SPEED_D, SPEED_F, SPEED_IZONE, SPEED_RAMPRATE, SPEED_PROFILE);
		rTalon.setPID(POSITION_P, POSITION_I, POSITION_D, POSITION_F, POSITION_IZONE, POSITION_RAMPRATE, POSITION_PROFILE);
		rTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rTalon.configEncoderCodesPerRev(360);
		
		rTalonSlave = new CANTalon(RobotMap.driveRightSlave);
		rTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonSlave.set(rTalon.getDeviceID());
		
		rTalonSlave2 = new CANTalon(RobotMap.driveRightSlave2);
		rTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonSlave2.set(rTalon.getDeviceID());
		
		shifter = new DoubleSolenoid(RobotMap.shifterSolenoidLow, RobotMap.shifterSolenoidHigh);
		shifter.set(Value.kForward);
		
		maxSpeed = SLOW_SPEED;
		
		robotDrive = new RobotDrive(lTalon, rTalon);
	}

	public void tankDrive(double left, double right)  {
		robotDrive.tankDrive(left, right);
	}
	
	public void shift(){
		if(shifter.isFwdSolenoidBlackListed()){
			shifter.set(Value.kReverse);
		}
		else{
			shifter.set(Value.kForward);
		}
	}
	
	public void closedLoopDrive(double left, double right){
		
		switch(controlMode){
		case Disabled:
		case Position:
			lTalon.set(left);
			rTalon.set(right);
			break;
		case Speed:
			lTalon.set(left * maxSpeed);
			rTalon.set(right * maxSpeed);
			SmartDashboard.putNumber("Left Set Speed", left * maxSpeed);
			SmartDashboard.putNumber("Right Set Speed", right * maxSpeed);
			break;
		default:
			break;
			
		}
	}
	
	public void setClosedLoopSpeed(){
		controlMode = CANTalon.TalonControlMode.Speed;
		
		lTalon.setProfile(SPEED_PROFILE);
		rTalon.setProfile(SPEED_PROFILE);
		
		lTalon.changeControlMode(controlMode);
		rTalon.changeControlMode(controlMode);
	}
	
	public void setClosedLoopPosition(){
		controlMode = CANTalon.TalonControlMode.Position;
		
		lTalon.setProfile(POSITION_PROFILE);
		rTalon.setProfile(POSITION_PROFILE);
		
		lTalon.changeControlMode(controlMode);
		rTalon.changeControlMode(controlMode);
	}
	
	public void toggleMaxSpeed(){
		if(maxSpeed == FAST_SPEED){
			maxSpeed = SLOW_SPEED;
		}
		else{
			maxSpeed = SLOW_SPEED;
		}
	}
	
	public void resetEncoders(){
		lTalon.setPosition(0);
		rTalon.setPosition(0);
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
		double speed = ((rTalon.getEncVelocity() / ENC_CYCLES_PER_REV * 10.0) * GEAR_RATIO	* WHEEL_DIAMETER * Math.PI / 12	+ 
						(lTalon.getEncVelocity() / ENC_CYCLES_PER_REV * 10.0) * GEAR_RATIO	* WHEEL_DIAMETER * Math.PI / 12) / 2;
		return speed;
	}
	
	public void smartDashboard(){
		SmartDashboard.putNumber("LeftEncoder: ", lTalon.getPosition());
		SmartDashboard.putNumber("LeftSpeed: ", lTalon.getSpeed());

		SmartDashboard.putNumber("RightEncoder: ", rTalon.getPosition());
		SmartDashboard.putNumber("RightSpeed: ", rTalon.getSpeed());
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveTank());
        // setDefaultCommand(new DriveArcade());
    }
}

