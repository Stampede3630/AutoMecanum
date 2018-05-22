package org.usfirst.frc.team3630.robot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//These imports are necissary for this specific implementation of the code, but not more generic versions.
import edu.wpi.first.wpilibj.Talon;


/*
 * The goal of this class is to handle the main functions of the drivetrain and to be able to be ported over into new code if necessary. 
 * This method handles all interactions with the drivetrain, which are the 4 motors and encoders that control robot movement through the mecanum drivetrain
 */
public class DriveTrain {
	
	//These are declared generically so that any speed controller can be used. This line does not need to be modified, but you can define your specific type of speed controller in the constructor (ie Talon, Victor etc)
	SpeedController fL, fR, rL, rR;
	MecanumDrive driveTrain;
	public DriveTrain () {
		
		//Constructions can change here
		fL = new Talon(1);
		fR = new Talon(2);
		rL = new Talon(3);
		rR = new Talon(4);
		driveTrain = new MecanumDrive(fL,rL,fR,rR);
	}
	
	

	
}
