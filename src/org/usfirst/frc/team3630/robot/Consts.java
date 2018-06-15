package org.usfirst.frc.team3630.robot;

public class Consts {
public final static double xKP= .05;
public final static double xKD= 0;
public final static double xKI= 0;


public final static  double yKP = .05;
public final static double yKI = 0;
public final  static double yKD = 0;

public final static  double thetaKP = .05;
public final static double thetaKI = 0;
public final  static double thetaKD = 0;
public static final int pulsesPerRevolution = 250;
public static final double wheelRadius = 4;


public static final int fLPwm = 0;
public static final int fRPwm = 1;
public static final int rLPwm = 2;
public static final int rRPwm = 2;


public static final double degreesToRadians =  Math.PI/180;
public static final double radiansToDegrees =  180/Math.PI;
}
