/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3749.robot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Robot extends IterativeRobot {
	// Camera thread
	Thread m_visionThread;

	// accesses controller
	private Joystick stick;
	
	// drive motors
	private SpeedController leftMotor1;
	private SpeedController leftMotor2;
	private SpeedController rightMotor1;
	private SpeedController rightMotor2;
	
	// speed controller groups for drive
	private SpeedControllerGroup m_left;
	private SpeedControllerGroup m_right;
	
	// main drive object
	private DifferentialDrive drive;
	
	// accesses the accelerometer in the RoboRIO
	private Accelerometer accel;
	
	private SpeedController gearMotor;
	
	// stores data for the accelerometer, velocity, and distance
	private double [] accelData;
	private double [] velData;
	private double [] distData;
	
	// milliseconds of last frame
	private long pMillis;
	// delta time, time since last frame
	private long dTime;
	
	private boolean autoDone;
	
	// which drive control, two joystick or one joystick
	private final boolean ONE_JOYSTICK = false;
	
	// how much to scale down general speeds
	private final double scalePower = 0.6;
	
	//autonomous speed constants (to fix left side being slow)
	private final double leftSpeed = 1; // multiply left speed
	private final double rightSpeed = .7; // multiply right speed
	
	// initializes the robot
	public void robotInit() {
		// creates a Joystick based on the controller at port 0
		stick = new Joystick(0);
		
		// the drive motors (Spark/Talon)
		leftMotor1 = new Spark(0);
		leftMotor2 = new Talon(1);
		rightMotor1 = new Spark(3);
		rightMotor2 = new Talon(2);
		
		// groups of motors
		m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
		m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
		
		// creates drive system based on let and right speed controller groups
		drive = new DifferentialDrive(m_left, m_right);
		
		// gearMotor = new Talon (9);
		
		// get the accelerometer onboard the RoboRIO
		accel = new BuiltInAccelerometer();
		accelData = new double [3];
		velData = new double [3];
		distData = new double [3];
		
		pMillis = System.currentTimeMillis();
		
		/*
		 * camera sample code
		 * radio must be reconfigured in order for it to work
		 */
		
/*		m_visionThread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera
					= CameraServer.getInstance().addAxisCamera("axis-camera.local");
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream
					= CameraServer.getInstance().putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
						new Scalar(255, 255, 255), 5);
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();
*/	}
	
	@Override
	public void autonomousInit() {
		resetAccel();
		autoDone = false;
	}
	
	public void autonomousDisable() {
		resetAccel ();
		autoDone = false;
	}
	
	@Override
	public void autonomousPeriodic() {
		
		while (isEnabled() && !autoDone)
		{
		//	System.out.println("Accelerometer data: " + accelData[0] + ", " + accelData[1] + ", " + accelData[2]);
		//	System.out.println("Velocity data: " + velData[0] + ", " + velData[1] + ", " + velData[2]);
		//	System.out.println("Distance data: " + distData[0] + ", " + distData[1] + ", " + distData[2]);
			
			forwardTime (5);
			autoDone = true;
		}
		
	}

	@Override
	public void teleopPeriodic() {
		// console debugging 
		
		while (isOperatorControl() && isEnabled()) {
			
			// System.out.println("Right joystick y: " + (-stick.getRawAxis(5)));
			
			if (ONE_JOYSTICK)
			{
				/*
				 * one joystick control
				 * gets the y (and inverts) and the x 
				 */
				
				// ensures at least one joystick is out
				if (Math.abs (stick.getRawAxis(0)) > 0.01 || Math.abs(stick.getRawAxis(1)) > 0.01)
					drive.arcadeDrive(-stick.getRawAxis(1) * scalePower, stick.getRawAxis(0) * scalePower);
				else
					drive.arcadeDrive(0, 0); // otherwise reset
			}
			else
			{
				/*
				 * two joystick control
				 * gets the y of the left and right joysticks and inverts
				 */
				if (Math.abs (stick.getRawAxis(5)) > 0.01 || Math.abs(stick.getRawAxis(1)) > 0.01)
					drive.tankDrive (-stick.getRawAxis (1) * scalePower / rightSpeed, -stick.getRawAxis (5) * scalePower / leftSpeed, true);
				else
					drive.tankDrive(0, 0);
			}
		}
	}

	@Override
	public void testPeriodic() {
		
		while (isEnabled()) {
			//Decide on which axis, experiment with it
			double speed = stick.getRawAxis(3) - stick.getRawAxis(2);
			// gearMotor.set (speed/2);
		}
	}
	public void updateAccel ()
	{
		// get accelerometer data from accelerometer object
		accelData[0] = accel.getX() * 9.81;
		accelData[1] = accel.getY() * 9.81;
		accelData[2] = accel.getZ() * 9.81;
		
		dTime = System.currentTimeMillis() - pMillis;
		pMillis = System.currentTimeMillis();
		
		// accumulates velocity based on acceleration and time
		velData[0] += accelData[0] * dTime / 1000.0;
		velData[1] += accelData[1] * dTime / 1000.0;
		velData[2] += accelData[2] * dTime / 1000.0;

		// accumulates distance based on velocity and time
		distData[0] += velData[0] * dTime / 1000.0;
		distData[1] += velData[1] * dTime / 1000.0;
		distData[2] += velData[2] * dTime / 1000.0;
	}
	public void resetAccel ()
	{
		accelData = new double [3];
		velData = new double [3];
		distData = new double [3];
	}
	public void forward ()
	{
		drive.tankDrive(leftSpeed * 0.5, rightSpeed * 0.5);
	}
	public void forwardDist (double dist)
	{
		forwardTime (dist / 0.4116);
	}
	public void forwardTime (double seconds)
	{
		double start = System.currentTimeMillis();
		
		while ((System.currentTimeMillis() - start) < seconds * 1000)
		{
			System.out.println (System.currentTimeMillis() - start);
			forward ();
			Timer.delay(0.01);
		}
	}
	/*
	 * methods to implement for autonomous
	 * 
	 * public void forward (double power);
	 * public void forwardDist (double power, double dist);
	 * public void forwardTime (double seconds);
	 * public void turnRight (double degrees);
	 * public void turnLeft (double degrees);
	 */	
	
}
