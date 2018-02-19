package org.usfirst.frc.team3749.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * class AutoDrive supplies methods for driving for a certain distance and turning a 
 * set amount of degrees
 * @author Team Optix
 *
 */
public class AutoDrive {
	private DifferentialDrive drive;
	private Gyro gyro;
	
	// whether to go arcade drive or tank drive
	private final boolean arcadeDrive = true;

	// how much to scale down general speeds
	private final double scalePower = 0.85;
	
	// autonomous speed constants for tank drive
	private final double leftSpeed = 1; // multiply left speed
	private final double rightSpeed = 1; // multiply right speed

	// autonomous speed constants for arcade drive
	private double ySpeed = 1;
	private double xSpeed = 1;
	
	/**
	 * constructor AutoDrive creates a autonomous driving object based on the drive system and a gyro
	 * @param drive - the main driving system
	 * @param gyro - the gyro sensor (used for turns / driving straight)
	 */
	public AutoDrive (DifferentialDrive drive, Gyro gyro)
	{
		this.drive = drive;
		this.gyro = gyro;
	}
	/**
	 * method forward uses either arcade or tank drive to move forward, based on calibration variables
	 * @param speed - how fast to go (0 to 1)
	 */
	public void forward (double speed)
	{
		if (arcadeDrive)
			drive.arcadeDrive(ySpeed * scalePower * Math.abs(speed), xSpeed * scalePower * Math.abs(speed));
		else
			drive.tankDrive(leftSpeed * speed, rightSpeed * speed);
	}
	/**
	 * method forwardDist goes forward for a certain distance
	 * 
	 * @param dist is how far to go in inches
	 */
	public void forwardDist (double dist)
	{
		/*
		 * to calibrate:
		   * see how far robot goes in 5 seconds
		   * see how far robot goes in 0.5 seconds (slow start)
		   * for t > 0.5 constant speed, but t < 0.5 acceleration is constant (speed is increasing)
		 */
		double inches_per_5_seconds = 60; // FIX THIS
		double inches_per_half_second = 10; // FIX THIS
		double inches_per_second = (inches_per_5_seconds - inches_per_half_second) / 4.5; // account for the slow start
		
		// goes forward
		
		// if time would be less than 0.5. the speed vs time would be the 1/2 * x^2 graph (calculus)
		// thus a sqrt is used to invert (algebra)
		if (dist < inches_per_half_second)
			forwardTime (0.5 * Math.sqrt(dist/inches_per_half_second));
		else
			forwardTime ((dist - inches_per_half_second) / inches_per_second + 0.5); // to account for the slow-down
	}
	/**
	 * method forwardTime goes forward for a certain amount of time
	 * @param seconds how many seconds to move forward for
	 */
	public void forwardTime (double seconds)
	{
		// milliseconds when started
		double start = System.currentTimeMillis();
		
		// as long as the delta time from start to current time is less than seconds * 1000 (to milliseconds)
		while ((System.currentTimeMillis() - start) < seconds * 1000)
		{
			// move forward, but gradually
			// max speed at 0.75
			forward (Math.min((System.currentTimeMillis() - start) / 500, 0.75));
			Timer.delay(0.01);
		}
	}
	/**
	 * method turn - turns the robot. negative degrees means left, positive means right
	 * @param degrees to turn (left is negative)
	 */
	public void turn (double degrees)
	{
		// use a protractor
		int degrees_per_second = 120; // FIX THIS
		turnTime (degrees < 0, degrees / degrees_per_second);
	}
	/**
	 * method turnTime - turns the robot for an amount of seconds
	 * @param left - whether the robot should turn left
	 * @param seconds - how long to turn for
	 */
	public void turnTime (boolean left, double seconds)
	{
		// milliseconds when started
		double start = System.currentTimeMillis();
		
		// as long as the delta time from start to current time is less than seconds * 1000 (to milliseconds)
		while ((System.currentTimeMillis() - start) < seconds * 1000)
		{
			// turn but don't go forward, left means negative
			drive.arcadeDrive (0, (left ? -1 : 1) * 0.4);

			Timer.delay(0.01);
		}
	}
}
