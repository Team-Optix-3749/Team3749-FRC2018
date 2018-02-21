/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3749.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Robot class controls robot for teleop and autonomous for Team 3749.
 * 
 * @author Team Optix
 * @date February 18, 2018
 */
public class Robot extends IterativeRobot {
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
	
	private AutoDrive auto;
	
	// if autonomous is completed
	private boolean autoDone;
	
	// which drive control, two joystick or one joystick
	private boolean arcadeDrive = true;
	
	// how much to scale down general speeds
	private final double scalePower = 0.85;
	
	// autonomous speed constants for tank drive
	private final double leftSpeed = 1; // multiply left speed
	private final double rightSpeed = 1; // multiply right speed

	// autonomous speed constants for arcade drive
	private double ySpeed = 1;
	private double xSpeed = 1.1; // goes a bit more right
	
	private double encoderScale = 1;
	
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
		armMotor = new TalonSRX (42);
		leftFly = new Spark (4);
		rightFly = new Spark (5);
		
		// creates an autonomous drive system with no gyro
		auto = new AutoDrive (drive, null);
	}
	
	@Override
	public void autonomousInit() {
		// so that autonomous runs
		autoDone = false;
		
		// sets the encoder to 0
		reset();
		
		// sets encoder distance units to degrees FIX THIS
		encoderScale = 1;
	}
	
	@Override
	public void autonomousPeriodic() {
		
		while (isEnabled() && !autoDone)
		{	
			// gets the game data, a string like "LRR" which gives the positions of the alliance's switches/scale
			String gameData = DriverStation.getInstance().getGameSpecificMessage();
			
			// what position the robot will start at (left = 0, middle = 1, right = 2)
			int roboPos = 0;
			
			// activate FLL level encapsulation
			
			// there are 3 places the robot can start
			switch (roboPos)
			{
				case 0: // left
				   	if(gameData.charAt(0) == 'L') // our switch on left
				   	{
						// move to the box release position
						setArm(90);
				   		auto.forwardDist(140);
				   		auto.turn(90);
				   		releaseBox(0.6);
						// get out of the way
						auto.turn(-90);
						auto.forwardDist(100);
				   	}
				   	else if (gameData.charAt(0) == 'R') // our switch on right
				   	{
						// move to the box release position
						setArm(90);
				   		auto.forwardDist(230);
				   		auto.turn(90);
				   		auto.forwardDist(140);
				   		auto.turn(90);
				   		releaseBox(1);
				   	}
					break;
				case 2: // right
				   	if(gameData.charAt(0) == 'R') // our switch on right
				   	{
				   		// move to the box release position
						setArm(90);
				   		auto.forwardDist(140);
				   		auto.turn(-90);
				   		releaseBox(0.6);
				   		// get out of the way
				   		auto.turn(-90);
				   		auto.forwardDist(100);
				   	}
				   	else if (gameData.charAt(0) == 'L') // our switch on left
				   	{
						setArm(90);
				   		auto.forwardDist(230);
				   		auto.turn(-90);
				   		auto.forwardDist(140);
				   		auto.turn(-90);
				   		releaseBox(1);
				   	}
					break;
				case 1: // middle
				   	if(gameData.charAt(0) == 'R') // our switch on right
				   	{
						setArm(90);
				   		auto.forwardDist(20);
				   		auto.turn(10);
				   		auto.forwardDist(60);
				   		auto.turn(-10);
				   		releaseBox(1);
				   		auto.turn(90);
				   		auto.forwardDist(60);
				   		auto.turn(-90);
				   		auto.forwardDist(80);
				   	}
				   	else if (gameData.charAt(0) == 'L') // our switch on left
				   	{
						setArm(90);
				   		auto.forwardDist(60);
				   		auto.turn(-90);
				   		auto.forwardDist(90);
				   		auto.turn(90);
				   		auto.forwardDist(60);
				   		auto.turn(90);
				   		releaseBox(1);
				   	}
					break;
			}
			// please don't keep running through autonomous when done :P
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
				drive.arcadeDrive(-stick.getRawAxis(1) * ySpeed * scalePower, stick.getRawAxis(4) * xSpeed * scalePower, true);
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
			
			// other motors
			
			// sets main arm to half of speed given from left/right triggers
			
			speed = (stick.getRawAxis(2) - stick.getRawAxis(3)) * 0.5;
			
			// limit the absolute value of speed  to 0.48
			armMotor.set(ControlMode.PercentOutput, speed);
			
			// if no input
			if (speed == 0)
				armMotor.set(ControlMode.PercentOutput, -getRate() / 1000000);
			/*
			if (getAngle() < 0 && speed < 0)
				armMotor.set(ControlMode.PercentOutput, 0);
			if (getAngle() > 120 && speed > 0) // FIX THIS
				armMotor.set(ControlMode.PercentOutput, 0);
			*/
			System.out.println("Angle: " + getAngle()/12000);
			System.out.println("Velocity: " + -getRate()/1000000);
			
			// sets speed if only one bumper button
			double flySpeed = 0;
			if (stick.getRawButton(5) && !stick.getRawButton(6))
				flySpeed = 0.8;
			if (stick.getRawButton(6) && !stick.getRawButton(5))
				flySpeed = -1;
			
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
	 * method releaseBox shoots the box out by settings the flywheels power for 2 seconds
	 * @param power - how fast to shoot (0 to 1)
	 */
	private void releaseBox (double power)
	{
		// milliseconds when started
		double start = System.currentTimeMillis();
		
		// as long as the delta time from start to current time is less than 2000ms, 2s
		while ((System.currentTimeMillis() - start) < 2000)
		{
			// negative means out
			leftFly.set(-1 * Math.abs(power));
			rightFly.set(Math.abs(power));
			Timer.delay(0.01);
		}
	}
	/**
	 * method setArm - sets an arm to a specific position from the encoder, 0 is upright, directly forward is 90
	 * @param degrees - which angle to be at, 0 = vertical
	 */
	private void setArm (double degrees)
	{
		// if current position is too big, move negatively
		int direction = getAngle() > degrees ? -1 : 1;
		double start = System.currentTimeMillis();
		
		// restrict to the endpoints
		degrees = Math.min(Math.max(degrees, 0), 120); // fix this
		
		// as long as the angle and the target angle is greater than 5 away
		while (Math.abs(getAngle() - degrees) > 5)
		{
			armMotor.set(ControlMode.PercentOutput, direction * Math.min((System.currentTimeMillis() - start)/2000, 0.4));
			Timer.delay(0.01);
		}
	}
	/**
	 * method getAngle - gets the angle  of the motor
	 * @return angle
	 */
	private double getAngle ()
	{
		return armMotor.getSelectedSensorPosition(0) * encoderScale;
	}
	/**
	 * method getRate - gets the rate of the main arm motor, but converted to degrees/second
	 * @return the rate in degrees
	 */
	private double getRate ()
	{
		return armMotor.getSelectedSensorVelocity(0) * encoderScale;
	}
	/**
	 * method reset - sets the encoder value to 0
	 */
	private void reset()
	{
		armMotor.setSelectedSensorPosition(0, 0, 100);
	}
}
