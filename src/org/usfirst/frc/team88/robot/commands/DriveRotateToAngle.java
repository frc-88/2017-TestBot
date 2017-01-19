package org.usfirst.frc.team88.robot.commands;

import java.io.*;

import org.usfirst.frc.team88.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveRotateToAngle extends Command {
	double targetAngle;
    private int count;
    private File file;
    private FileWriter fileWriter;
    private BufferedWriter bufferedWriter;
	
    public DriveRotateToAngle(double angle) {
        requires(Robot.drive);

        targetAngle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.drive.setOpenLoop();
        count = 0;
        //initLogFile();
    	Robot.drive.disableRampRate();
    	Robot.drive.rotateController.reset();
    	Robot.drive.rotateController.setSetpoint(targetAngle);
    	Robot.drive.rotateController.enable();    	
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.smartDashboard(0);
    	//writeLog(count++ + "," + Robot.drive.getStatus() + "\n");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		SmartDashboard.putNumber("Rot Error", Robot.drive.rotateController.getError());
		SmartDashboard.putNumber("Rot On Target", Robot.drive.rotateController.onTarget()?1:0);

		return Robot.drive.rotateController.onTarget();
	}

    // Called once after isFinished returns true
    protected void end() {
        //closeLog();
		Robot.drive.rotateController.disable();
		//Robot.drive.setClosedLoopSpeed();
		Robot.drive.enableRampRate();
   }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        //closeLog();
		Robot.drive.rotateController.disable();    	
		//Robot.drive.setClosedLoopSpeed();
		Robot.drive.enableRampRate();
   }
    
    private void initLogFile() {
        try {
            file = new File("/home/lvuser/rotatelog.txt");
            if(!file.exists()){
                file.createNewFile();
            }
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        bufferedWriter = new BufferedWriter(fileWriter);
    }
    
    private void writeLog(String text) {
        try {
            bufferedWriter.write(text);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
    private void closeLog() {
        try {
            bufferedWriter.close();
            fileWriter.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
