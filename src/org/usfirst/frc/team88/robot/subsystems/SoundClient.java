package org.usfirst.frc.team88.robot.subsystems;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class SoundClient extends Subsystem {
    private static PrintWriter out;
    private static Socket echoSocket;
    
	public SoundClient() {
		// set things up code
        String hostName = "10.88.88.21";
        int portNumber = 5800;
 
        try {
            echoSocket = new Socket(hostName, portNumber);
            out = new PrintWriter(echoSocket.getOutputStream(), true);
            BufferedReader in =
                new BufferedReader(
                    new InputStreamReader(echoSocket.getInputStream()));
            BufferedReader stdIn =
                new BufferedReader(
                    new InputStreamReader(System.in));
        } catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                hostName);
            System.exit(1);
        }
	}
	
	public void play(String fileNickname) {
		// send the fileNickname to the SoundServer
        out.println(fileNickname);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

