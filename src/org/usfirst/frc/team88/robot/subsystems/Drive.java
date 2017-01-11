
package org.usfirst.frc.team88.robot.subsystems;

import org.usfirst.frc.team88.robot.Robot;
import org.usfirst.frc.team88.robot.RobotMap;
import org.usfirst.frc.team88.robot.commands.DriveTank;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem implements PIDOutput {
	public final static double ENC_CYCLES_PER_REV = 360.0;
	public final static double GEAR_RATIO = (3 * 34) / 50;
	public final static double WHEEL_DIAMETER = 4;

	private final static int LOW_PROFILE = 0;
	private final static double LOW_RAMPRATE = 60;
	private final static double LOW_P = 0.0;
	private final static double LOW_I = 0.0;
	private final static double LOW_D = 0.0;
	private final static double LOW_F = 0.5;
	private final static int LOW_IZONE = 0;
	private final static double LOW_MAX = 700;

	private final static int HIGH_PROFILE = 1;
	private final static double HIGH_RAMPRATE = 60;
	private final static double HIGH_P = 0.0;
	private final static double HIGH_I = 0.0;
	private final static double HIGH_D = 0.0;
	private final static double HIGH_F = 0.5;
	private final static int HIGH_IZONE = 0;
	private final static double HIGH_MAX = 1400;

	private final static double DIFF_MAX = (HIGH_MAX - LOW_MAX)/100 + 1;

	private final static double ROTATE_P = 0.01;
	private final static double ROTATE_I = 0.0002;
	private final static double ROTATE_D = 0.0;
	private final static double ROTATE_F = 0.0;
	private final static double ROTATE_TOLERANCE = 4.0f;
	
	public PIDController rotateController;
	
	private final CANTalon lTalon, lTalonFollower, lTalonFollower2, rTalon, rTalonFollower, rTalonFollower2;
	private DoubleSolenoid shifter;
	private AHRS navx;

	private double maxSpeed;
	private double targetMaxSpeed;
	private double speedIncrement;
	private boolean autoShift;
	private CANTalon.TalonControlMode controlMode;

	public Drive() {
		// init left master
		lTalon = new CANTalon(RobotMap.driveLeft);
		lTalon.setPID(LOW_P, LOW_I, LOW_D, LOW_F, LOW_IZONE, LOW_RAMPRATE, LOW_PROFILE);
		lTalon.setPID(HIGH_P, HIGH_I, HIGH_D, HIGH_F, HIGH_IZONE, HIGH_RAMPRATE, HIGH_PROFILE);
		lTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lTalon.configEncoderCodesPerRev(360);
		lTalon.configNominalOutputVoltage(+0.0f, -0.0f);
		lTalon.configPeakOutputVoltage(+12.0f, -12.0f);
		lTalon.reverseSensor(false);
		lTalon.reverseOutput(false);
		lTalon.enableBrakeMode(false);
		lTalon.setVoltageRampRate(LOW_RAMPRATE);

		// init left followers
		lTalonFollower = new CANTalon(RobotMap.driveLeftSlave);
		lTalonFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonFollower.set(lTalon.getDeviceID());
		lTalonFollower2 = new CANTalon(RobotMap.driveLeftSlave2);
		lTalonFollower2.changeControlMode(CANTalon.TalonControlMode.Follower);
		lTalonFollower2.set(lTalon.getDeviceID());

		// init rigbt master
		rTalon = new CANTalon(RobotMap.driveRight);
		rTalon.setPID(LOW_P, LOW_I, LOW_D, LOW_F, LOW_IZONE, LOW_RAMPRATE, LOW_PROFILE);
		rTalon.setPID(HIGH_P, HIGH_I, HIGH_D, HIGH_F, HIGH_IZONE, HIGH_RAMPRATE, HIGH_PROFILE);
		rTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rTalon.configEncoderCodesPerRev(360);
		rTalon.configNominalOutputVoltage(+0.0f, -0.0f);
		rTalon.configPeakOutputVoltage(+12.0f, -12.0f);
		rTalon.reverseSensor(true);
		rTalon.reverseOutput(false);
		rTalon.enableBrakeMode(false);
		rTalon.setVoltageRampRate(LOW_RAMPRATE);

		// init right followers
		rTalonFollower = new CANTalon(RobotMap.driveRightSlave);
		rTalonFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonFollower.set(rTalon.getDeviceID());
		rTalonFollower2 = new CANTalon(RobotMap.driveRightSlave2);
		rTalonFollower2.changeControlMode(CANTalon.TalonControlMode.Follower);
		rTalonFollower2.set(rTalon.getDeviceID());

		// init shifter
		shifter = new DoubleSolenoid(RobotMap.shifterSolenoidLow, RobotMap.shifterSolenoidHigh);
		shifter.set(Value.kForward);
		autoShift = true;
		maxSpeed = LOW_MAX;
		targetMaxSpeed = LOW_MAX;
		lTalon.setProfile(LOW_PROFILE);
		rTalon.setProfile(LOW_PROFILE);
		
		// init navx
		navx = new AHRS(SerialPort.Port.kMXP);
		
		// init rotateController
		rotateController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D, ROTATE_F, navx, this);
		rotateController.setInputRange(-180.0f, 180.0f);
		rotateController.setOutputRange(-1.0, 1.0);
		rotateController.setAbsoluteTolerance(ROTATE_TOLERANCE);
		rotateController.setContinuous(true);
		
		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("Drive", "RotateController", rotateController);
	}

	public void shift(){
		if(shifter.get() == Value.kForward){
			shifter.set(Value.kReverse);
			targetMaxSpeed = HIGH_MAX;
			lTalon.setProfile(HIGH_PROFILE);
			rTalon.setProfile(HIGH_PROFILE);
		}
		else{
			shifter.set(Value.kForward);
			targetMaxSpeed = LOW_MAX;
			lTalon.setProfile(LOW_PROFILE);
			rTalon.setProfile(LOW_PROFILE);
		}
	}

	public boolean isLowGear(){
		if(shifter.get() == Value.kForward){
			return true;
		}
		else 
			return false;
	}

	public void setTarget(double left, double right){

		SmartDashboard.putNumber("Left input", left);
		SmartDashboard.putNumber("Right input", right);

		switch(controlMode){
		case Disabled:
		case PercentVbus:
		case Position:
			lTalon.set(left);
			rTalon.set(-right);
			break;
		case Speed:
			lTalon.set(left * getMaxSpeed());
			rTalon.set(right * getMaxSpeed());
			SmartDashboard.putNumber("Left Set Speed", left * getMaxSpeed());
			SmartDashboard.putNumber("Right Set Speed", right * getMaxSpeed());
			break;
		default:
			break;

		}
	}

	public void setOpenLoop(){
		controlMode = CANTalon.TalonControlMode.PercentVbus;

		lTalon.changeControlMode(controlMode);
		rTalon.changeControlMode(controlMode);
	}

	public void setClosedLoopSpeed(){
		controlMode = CANTalon.TalonControlMode.Speed;

		lTalon.changeControlMode(controlMode);
		rTalon.changeControlMode(controlMode);
	}

	private double getMaxSpeed() {
		if ( (targetMaxSpeed - maxSpeed > DIFF_MAX ) || (targetMaxSpeed - maxSpeed < -DIFF_MAX) ) {
			if (speedIncrement == 0) {
				speedIncrement = (targetMaxSpeed - maxSpeed) / 100;
			}
			maxSpeed += speedIncrement;
		} else {
			speedIncrement = 0;
			maxSpeed = targetMaxSpeed;
		}

		return maxSpeed;
	}

	public void resetEncoders(){
		lTalon.setPosition(0);
		rTalon.setPosition(0);
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
		double speed = (lTalon.getSpeed() + rTalon.getSpeed()) / 2;

		return speed;
	}

	public boolean isAutoShift() {
		return autoShift;
	}

	public void toggleAutoShift() {
		autoShift = !autoShift;
	}

	public void smartDashboard(int state){
		SmartDashboard.putNumber("LeftEncoder: ", lTalon.getPosition());
		SmartDashboard.putNumber("LeftSpeed: ", lTalon.getSpeed());
		SmartDashboard.putNumber("LeftEncVel: ", lTalon.getEncVelocity());
		
		SmartDashboard.putNumber("RightEncoder: ", rTalon.getPosition());
		SmartDashboard.putNumber("RightSpeed: ", rTalon.getSpeed());
		SmartDashboard.putNumber("RightEncVel: ", rTalon.getEncVelocity());

		SmartDashboard.putNumber("LeftError: ", lTalon.getClosedLoopError());
		SmartDashboard.putNumber("RightError: ", rTalon.getClosedLoopError());

		SmartDashboard.putNumber("ShifterState: ", state);

		SmartDashboard.putNumber("targetMaxspeed", targetMaxSpeed);
		SmartDashboard.putNumber("maxSpeed", getMaxSpeed());

		// for Network Tables stuff
		SmartDashboard.putBoolean("lowGear", isLowGear());
		SmartDashboard.putNumber("leftCurrent", lTalon.getOutputCurrent());
		SmartDashboard.putNumber("rightCurrent", rTalon.getOutputCurrent());

		//NavX stuff
		SmartDashboard.putBoolean("IMU_Connected", navx.isConnected()); 
		SmartDashboard.putBoolean("IMU_IsCalibrating", navx.isCalibrating()); 
		SmartDashboard.putNumber("IMU_Yaw", navx.getYaw()); 
		SmartDashboard.putNumber("IMU_Pitch", navx.getPitch()); 
		SmartDashboard.putNumber("IMU_Roll", navx.getRoll()); 

		SmartDashboard.putNumber("Displacement_X", navx.getDisplacementX()); 
		SmartDashboard.putNumber("Displacement_Y", navx.getDisplacementY());
		
		// NetworkTable stuff
		SmartDashboard.putNumber("NT_Distance", Robot.oiNetTable.table.getNumber("Distance",0.0));
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveTank());
		// setDefaultCommand(new DriveArcade());
	}

	@Override
	public void pidWrite(double output) {
		double max = 0.4;
		double min = 0.075;
		
		if (output > max) {
			output = max;
		} else if (output < min && output >0) {
			output = min;
		} else if (output == 0) {
			output = 0;
		} else if (output > (0-min)) {
			output = 0 - min;
		} else if (output < (0-max)) {
			output = 0 - max;
		}
		
		setTarget(-output, output);
	}
}

