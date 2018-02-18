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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
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

/**
 * Robot class controls robot for teleop and autonomous for Team 3749.
 * 
 * @author Team Optix
 * @date February 10, 2018
 */
public class Robot extends IterativeRobot {
	// Camera thread
	// Thread m_visionThread;

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
	
	// main drive control
	private DifferentialDrive drive;

	// the main arm
	private TalonSRX armMotor;
	
	// left and right flywheels
	private SpeedController leftFly;
	private SpeedController rightFly;

	// the encoder on the main arm
	private Encoder encoder;
	
	// position the arm needs to be at
	private double targetPos = 0;
	// max input for the arm
	private double optSpeed = 0.48;
	
	// if autonomous is completed
	private boolean autoDone;
	
	// which drive control, two joystick or one joystick
	private boolean arcadeDrive = true;
	private boolean previousState = false;
	
	// how much to scale down general speeds
	private final double scalePower = 0.85;
	
	// autonomous speed constants for tank drive
	private final double leftSpeed = 1; // multiply left speed
	private final double rightSpeed = 1; // multiply right speed

	// autonomous speed constants for arcade drive
	private double ySpeed = 1;
	private double xSpeed = .85; // goes a bit left more
	
	/**
	 * method robotInit is run to initialize the robot at the very beginning, used like a constructor
	 */
	@Override
	public void robotInit() {
		
		// creates a joystick based on the xbox controller locked at port 0
		stick = new Joystick(0);
		
		// the drive motors (Spark/Talon)
		leftMotor1 = new Spark(8);
		leftMotor2 = new Spark(9);
		rightMotor1 = new Spark(6);
		rightMotor2 = new Spark(7);
		
		// groups of motors
		m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
		m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
		
		// creates drive system based on let and right speed controller groups
		drive = new DifferentialDrive(m_left, m_right);
		
		// arm and flywheels
		armMotor = new TalonSRX (3);
		leftFly = new Spark (4);
		rightFly = new Spark (5);
		
		// encoder on the arm
		encoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		
		// get the accelerometer on the RoboRIO
		// accel = new BuiltInAccelerometer();
		
		// pMillis = System.currentTimeMillis();
	}
	
	@Override
	public void autonomousInit() {
		// so that autonomous runs
		autoDone = false;
		
		// sets the encoder to 0
		encoder.reset();
		
		// sets encoder distance units to degrees FIX THIS
		encoder.setDistancePerPulse (10);
	}
	
	@Override
	public void autonomousPeriodic() {
		
		while (isEnabled() && !autoDone)
		{	
			// forward for 5 seconds
			forwardTime (5);
			autoDone = true;
		}
		
	}

	@Override
	public void teleopInit () {
		
	}
	
	@Override
	public void teleopPeriodic() {

		double speed;
		while (isOperatorControl() && isEnabled()) {
			
			if (arcadeDrive)
			{
				// reset first in case joysticks are broke
				drive.arcadeDrive(0, 0);
				
				/*
				 * controls a two joystick drive
				 * left joystick y for forward/backward and right joystick x for left/right
				 */
				drive.arcadeDrive(-stick.getRawAxis(1) * ySpeed * scalePower, stick.getRawAxis(4) * xSpeed * scalePower * 0.8, true);
			}
			else
			{
				// resets if joysticks are broke
				drive.tankDrive(0, 0);
				
				/*
				 * two joystick control
				 * gets the y of the left and right joysticks and inverts
				 * divides by opposite power (instead of multiplying) to stay at fastest speed possible
				 */
				drive.tankDrive (-stick.getRawAxis (1) * scalePower / rightSpeed, -stick.getRawAxis (5) * scalePower / leftSpeed, true);
			}
			
//			if (!stick.getRawButton(2) && previousState)
//				arcadeDrive = !arcadeDrive;
//			previousState = stick.getRawButton(2);
			
			// other motors
			
			// sets main arm to half of speed given from left/right triggers
			
			speed = (stick.getRawAxis(3) - stick.getRawAxis(2)) * 0.8;
			
			// limit the absolute value of speed  to 0.48
			if (speed > 0.5)
				armMotor.set(ControlMode.PercentOutput, 0.5);
			else if (speed < -0.5)
				armMotor.set(ControlMode.PercentOutput, -0.5);
			else
				armMotor.set(ControlMode.PercentOutput, speed);
			
			if (encoder.getDistance() < 0 && speed < 0)
				armMotor.set(ControlMode.PercentOutput, 0);
//			if (encoder.getDistance() > 120 && speed > 0) FIX THIS
//				armMotor.set(ControlMode.PercentOutput, 0);
			
			System.out.println(speed);
			
			// sets speed if only one bumper button
			double flySpeed = 0;
			if (stick.getRawButton(5) && !stick.getRawButton(6))
				flySpeed = -0.5;
			if (stick.getRawButton(6) && !stick.getRawButton(5))
				flySpeed = 0.5;
			
			// negates right side (motors are upside down)
			leftFly.set(flySpeed);
			rightFly.set(-flySpeed);
			
			Timer.delay(0.01);
		}
	}

	@Override
	public void testPeriodic() {
		
		while (isEnabled()) {
			Timer.delay(0.01);
		}
		
	}
	/**
	 * method forward uses either arcade or tank drive to move forward, based on calibration variables
	 */
	private void forward ()
	{/*
		if (arcadeDrive)
			drive.arcadeDrive(ySpeed * scalePower * 0.5, xSpeed * scalePower * 0.5);
		else
			drive.tankDrive(leftSpeed * 0.5, rightSpeed * 0.5);*/
	}
	/**
	 * method forwardDist goes forward for a certain distance
	 * 
	 * @param dist is how far to go in inches
	 */
	private void forwardDist (double dist)
	{
		/*
		 * to calibrate:
		   * see how far robot goes in 5 seconds
		   * robot goes X inches per 5 seconds or X/5 inches/sec
		   * dist / (X/5) would be the appropriate time
		 */
		double inches_per_5_seconds = 60;
		double inches_per_second = inches_per_5_seconds / 5;
		
		// goes forward
		forwardTime (dist / inches_per_second);
	}
	/**
	 * method forwardTime goes forward for a certain amount of time
	 * @param seconds how many seconds to move forward for
	 */
	private void forwardTime (double seconds)
	{
		// milliseconds when started
		double start = System.currentTimeMillis();
		
		// as long as the delta time from start to current time is less than seconds * 1000 (to milliseconds)
		while ((System.currentTimeMillis() - start) < seconds * 1000)
		{
			// move forward
			forward ();
			Timer.delay(0.01);
		}
	}
	/*
	 * methods to implement for autonomous
	 * public void turnRight (double degrees);
	 * public void turnLeft (double degrees);
	 */	
	
}
