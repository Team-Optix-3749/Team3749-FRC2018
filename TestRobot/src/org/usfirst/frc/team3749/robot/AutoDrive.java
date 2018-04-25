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
	private final boolean arcadeDrive = false;

	// how much to scale down general speeds
	private final double scalePower = 0.7;
	
	// autonomous speed constants for tank drive
	private final double leftSpeed = 1.12; // multiply left speed
	private final double rightSpeed = 1; // multiply right speed

	// autonomous speed constants for arcade drive
	private double xShift = 0.3;

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
	 * constructor AutoDrive creates a autonomous driving object based on the drive system without gyro
	 * @param drive - the main driving system
	 */
	public AutoDrive (DifferentialDrive drive)
	{
		this.drive = drive;
		this.gyro = null;
	}
	/**
	 * method forward uses either arcade or tank drive to move forward, based on calibration variables
	 * @param speed - how fast to go (0 to 1)
	 */
	public void forward (double speed)
	{
		  if (arcadeDrive)
			drive.arcadeDrive(scalePower * Math.abs(speed), xShift);
		else
			drive.tankDrive(leftSpeed * speed * scalePower , rightSpeed * speed * scalePower);
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
		double inches_per_5_seconds = 94;
		double inches_per_half_second = 6;
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
		double degrees_per_second = 70.88 * 1.32;
		turnTime (degrees < 0, Math.abs(degrees) / degrees_per_second);
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
			drive.arcadeDrive (0, (left ? -1 : 1) * 0.7);

			Timer.delay(0.01);
		}
	}
}
