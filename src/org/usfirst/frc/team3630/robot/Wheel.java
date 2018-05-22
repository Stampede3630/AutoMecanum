package org.usfirst.frc.team3630.robot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Wheel {
	private static final int pulsesPerRevolution = 250;
	public static final double wheelRadius = 4; //in inches, or whatever units you are using
	XboxController xbox;
	public Encoder encoder;
	public Talon talon;
	
	
	/**
	 * @param encoderChannel1
	 * @param encoderChannel2
	 * @param talonChannel
	 * @param reversed
	 */

	public Wheel (int encoderChannel1, int encoderChannel2, int talonChannel, boolean reversed){
		xbox = new XboxController(0);
		encoder = new Encoder(encoderChannel1, encoderChannel2, reversed, EncodingType.k4X);
		
		talon = new Talon(talonChannel);		
		talon.setInverted(reversed);
		encoder.setMaxPeriod(1);
		// Define distance in terms of inches
		encoder.setDistancePerPulse(wheelRadius*2*Math.PI/pulsesPerRevolution);
		encoder.setMinRate(10);
		encoder.setReverseDirection(!reversed);
		encoder.setSamplesToAverage(7);
		encoder.setPIDSourceType(PIDSourceType.kDisplacement);

		
		//PID WILL ONLY DEAL WITH INCHES/SECOND
	}
	
	

	
	public void getInfo () {
		SmartDashboard.putNumber("PID Encoder Input"+String.valueOf(talon.getChannel()), encoder.pidGet());	
	}
	

	public double getDistDegrees()
	{
		return encoder.getDistance() * 180.0 / Math.PI; 
	}

	public double getDistRadians()
	{
		return encoder.getDistance(); 
	}

	/**
	 * @return encoder distance in inches
	 */
	public double getDistInches()
	{
		return getDistRadians() * wheelRadius;
	}

}