/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3630.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	DriveTrain driveBase;
	
	
	
	public void robotInit() {
	driveBase = new DriveTrain();
	}

	@Override
	public void autonomousInit() {
		driveBase.enablePositionFinder();
	}

	@Override
	public void autonomousPeriodic() {
		driveBase.runPositionFinder();
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	public void teleopInit(){
		driveBase.disable();
	}
	@Override
	public void teleopPeriodic() {
		driveBase.teleopDrive();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	
	
	}
	
	
}
