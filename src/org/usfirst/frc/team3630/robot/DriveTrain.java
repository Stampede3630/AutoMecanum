package org.usfirst.frc.team3630.robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//These imports are necessary for this specific implementation of the code, but not more generic versions.
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;


/*
 * The goal of this class is to handle the main functions of the drivetrain and to be able to be ported over into new code if necessary. 
 * This method handles all interactions with the drivetrain, which are the 4 motors and encoders that control robot movement through the mecanum drivetrain
 */
public class DriveTrain {
	

	
	//These are declared generically so that any speed controller can be used. This line does not need to be modified, but you can define your specific type of speed controller in the constructor (ie Talon, Victor etc)
	SpeedController fL, fR, rL, rR;
	MecanumDrive driveTrain;
	XboxController xBox;
	public DriveTrain () {
		
		xBox = new XboxController(0);
		
		//Constructions can change here
		fL = new Talon(1);
		fR = new Talon(2);
		rL = new Talon(3);
		rR = new Talon(4);
		
		
		driveTrain = new MecanumDrive(fL,rL,fR,rR);
	}
	
	public void teleopDrive () {
		driveTrain.driveCartesian(
				xBox.getX(Hand.kLeft),
				xBox.getY(Hand.kRight), 
				xBox.getX(Hand.kRight), 
				0);
	}
	

	public Vector forwardMecanum(double frontLeft, double rearLeft, double frontRight, double rearRight) {
		double vX = (frontLeft + rearLeft + frontRight + rearRight) / 4;
		double vY = (-frontLeft + rearLeft - rearRight + frontRight) / 4;
		double vTheta = (-frontLeft - rearLeft + rearRight + frontRight) / (4 * 90); //I believe this is in degrees
		
		return new Vector (vX,vY,vTheta);
	}

	class Vector {
		public double x,y,theta;
		public Vector (double _x, double _y, double _theta){
			x=_x;
			y=_y;
			theta = _theta;
		}
	}
}
