package org.usfirst.frc.team3630.robot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * @author walsl
 * speed controllers in this class should be genereic 
 */
public class Wheel {
	// danger makes sure you double check pulses per revelution is correct
 //in inches, or whatever units you are using
	XboxController xbox;
	public Encoder encoder;
	public SpeedController talon;
	
	
	/**
	 * @param encoderChannel1
	 * @param encoderChannel2
	 * @param talonChannel
	 * @param reversed
	 */

	public Wheel (
				Encoder _encoder,
				SpeedController _talon
			){
		xbox = new XboxController(0);
		encoder = _encoder;
		
		talon = _talon;
		
		encoder.setMaxPeriod(1);
		// Define distance in terms of inches
		encoder.setDistancePerPulse(Consts.wheelRadius*2*Math.PI/Consts.pulsesPerRevolution);
		encoder.setMinRate(10);
		encoder.setSamplesToAverage(7);
		encoder.setPIDSourceType(PIDSourceType.kDisplacement);
	}
		
	
	public void getInfo () {
		//SmartDashboard.putNumber("PID Encoder Input"+String.valueOf(talon.getChannel()), encoder.pidGet());	
	}
	

	public double getDistDegrees()
	{
		return encoder.getDistance() * Consts.radiansToDegrees; 
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
		return getDistRadians() * Consts.wheelRadius;
	}

}