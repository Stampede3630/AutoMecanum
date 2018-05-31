package org.usfirst.frc.team3630.robot;
import com.kauailabs.navx.frc.AHRS;
import com.sun.prism.impl.ps.BaseShaderContext.SpecialShaderType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//These imports are necessary for this specific implementation of the code, but not more generic versions.
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;


/*
 * The goal of this class is to handle the main functions of the drivetrain and to be able to be ported over into new code if necessary. 
 * This method handles all interactions with the drivetrain, which are the 4 motors and encoders that control robot movement through the mecanum drivetrain
 */
/**
 * @author StampedeRobotics
 *
 */
public class DriveTrain {
	
	class Vector {
		public double x,y, omega;
		public Vector (double _x, double _y, double _omega){
			x=_x;
			y=_y;
			omega = _omega;
		}
		public Vector(){}
		public void reset (){
			x = 0;
			y = 0;
			omega = 0;
		}
	}
	
	//These are declared generically so that any speed controller can be used. This line does not need to be modified, but you can define your specific type of speed controller in the constructor (ie Talon, Victor etc)
	Wheel fL, fR, rL, rR;
	AHRS ahrs;
	MecanumDrive driveTrain;
	XboxController xBox;
	public Vector displacement = new Vector(0,0,0);
	public Timer periodTimer = new Timer();
	PIDFeeder x,y,theta;
	boolean isEnabled = false;
	public DriveTrain () {
		
		xBox = new XboxController(0);
		ahrs = new AHRS(SPI.Port.kMXP);
		
		
		
		//Constructions can change here
		fL = new Wheel(
				new Encoder (0,1),
				new Talon(0)
				);
		fR = new Wheel(
				new Encoder(2,3),
				new Talon(1)
				);
		rL = new Wheel(
				new Encoder(4,5),
				new Talon(2)
				);
		rR = new Wheel(
				new Encoder(6,7),
				new Talon(3)
				);
		
		driveTrain = new MecanumDrive(fL.talon,rL.talon,fR.talon,rR.talon);
	}
	public void enable(){
		isEnabled= true;
		periodTimer.start();
		ahrs.reset();
		//reset displacement
		displacement.reset();
	}
	
	public void disable(){
		isEnabled = false;
		periodTimer.stop();
	}
	
	public void teleopDrive () {
		driveTrain.driveCartesian(
				xBox.getX(Hand.kLeft),
				xBox.getY(Hand.kRight), 
				xBox.getX(Hand.kRight), 
				0);
	}
	


	/**
	 * @param robotSpeed The vector of the untranslated robot speed
	 * @param theta_degrees The angle in degrees of the robot at the time of the robot speed
	 * @return The translated robot speed relative to the field instead of the robot
	 * 
	 * This method translates the robot speed to match the speed relative to the field instead of the robot.
	 * 
	 */
	public Vector fieldTranslation(Vector robotSpeed, double theta_degrees){
		Vector transformed = new Vector();
		//converting the angle translations to radians. Note that the thetaY_radians variable is for manipulating the Y. For notes, please see the picture documentation.
		double theta_radians = theta_degrees * Math.PI/180;
		double thetaY_radians = (theta_degrees+90) * Math.PI/180;
		
		transformed.x = robotSpeed.x * Math.cos(theta_radians) + robotSpeed.y * Math.cos(thetaY_radians);
		transformed.y = robotSpeed.x * Math.sin(theta_radians) + robotSpeed.y * Math.sin(thetaY_radians);
		transformed.omega = robotSpeed.omega;		
		
		return transformed;
	}
	
	/**
	 * @param frontLeft
	 * @param rearLeft
	 * @param frontRight
	 * @param rearRight
	 * @return
	 */
	public Vector forwardMecanum(double frontLeft, double rearLeft, double frontRight, double rearRight) {
		double vX = (frontLeft + rearLeft + frontRight + rearRight) / 4;
		double vY = (-frontLeft + rearLeft - rearRight + frontRight) / 4;
		double vTheta = (-frontLeft - rearLeft + rearRight + frontRight) / (4 * 90); //I believe this is in degrees
		
		return new Vector (vX,vY,vTheta);
	}
	
	public Vector displacementConversion(Vector periodSpeed){
		Vector periodDisplacement = new Vector();
		double time = periodTimer.get();
		
		periodTimer.reset();
		periodTimer.start();
		
		periodDisplacement.x = periodSpeed.x * time;
		periodDisplacement.y = periodSpeed.y * time;
		periodDisplacement.omega = periodSpeed.omega * time;
		
		displacement.x += periodDisplacement.x;
		displacement.y += periodDisplacement.y;
		displacement.omega += periodDisplacement.omega;
		
		return displacement;
	}

	/**
	 * 
	 */
	public void runPositionFinder () {
		if(isEnabled){
		Vector pDisplacement;
		pDisplacement = forwardMecanum(
				fL.getDistInches(),
				rL.getDistInches(),
				fR.getDistInches(),
				rR.getDistInches()
				);
		pDisplacement = fieldTranslation(pDisplacement,ahrs.getAngle());
		pDisplacement = displacementConversion(pDisplacement);
		}
		else System.out.println("WARNING: position finder attempted to run while disabled.");
	}
	
	
	class PIDFeeder implements PIDSource {
		double vSource, dSource;
		PIDSourceType sourceType = PIDSourceType.kDisplacement;
		public PIDFeeder (double _vSource, double _dSource){
			vSource = _vSource;
			dSource = _dSource;
		}
		public void setPIDSourceType(PIDSourceType pidSource){
			sourceType = pidSource;
		}
		
		public PIDSourceType getPIDSourceType (){
			return sourceType;
		}
		
		public double pidGet() {
			if(sourceType.equals(PIDSourceType.kDisplacement)) return dSource;
			else return vSource;
		}
	}
	
}
