package frc.robot.subsystems.SwerveDrive;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

/**
 * Calculates the angle and speed of each 4 wheels based
 * on the input from two 2-axis joysticks.</br>
 * Kinematics from <a href = https://www.chiefdelphi.com/media/papers/2426)> Ether</a>
 * 
 * @author created by Unnas Hussain on 8/2/2017
 */
public class SwerveHelper {

	/*
	 * Math Conventions:
	 * 
	 * 
	 * Wheel ID Numbers:
	 * 0 = front_right;
	 * 1 = front_left;
	 * 2 = rear_left;
	 * 3 = rear_right;
	 * 
	 * 0.0 degrees is the wheels facing the front of the bot
	 * 
	 */
	
	
	protected static double[] wheelSpeed = new double[4];
	protected static double[] wheelAngle = new double[4];
	protected static double[] wheelAngleChange = new double[4];

	protected static AHRS m_gyro = null;

	private static boolean fieldCentric = true;
	private static double robotCentricOffsetRadians = 0;

	private static boolean useAngleToReverse = true;
	
	public static double getSpeed(WheelPosition position) {
		return wheelSpeed[position.wheelNumber];
	}

	public static double getAngle(WheelPosition position) {
		return wheelAngle[position.wheelNumber];
	}	

	public static double getAngleChange(WheelPosition position) {
		return wheelAngleChange[position.wheelNumber];
	}	

	public static boolean isFieldCentric() {
		return fieldCentric;
	}

	public static void calculate(double strafe, double forward, double rotate, double[] currentAngle){
		if (fieldCentric) {
			double gyroYawRad = Math.toRadians(getGyroYawDeg());
			double temp = forward * Math.cos(gyroYawRad) + strafe * Math.sin(gyroYawRad);
			strafe = -forward * Math.sin(gyroYawRad) + strafe * Math.cos(gyroYawRad);
			forward = temp;
		} else if (robotCentricOffsetRadians != 0) {
			double temp = forward * Math.cos(robotCentricOffsetRadians) + strafe*Math.sin(robotCentricOffsetRadians);
			strafe = -forward*Math.sin(robotCentricOffsetRadians) + strafe*Math.cos(robotCentricOffsetRadians);
			forward = temp;
		}

		/*
		 * Inverse Kinematics Based on work from Ether on Cheif Delphi
		 */
		double frontRightX,frontRightY,backLeftX,backLeftY;
		double L = Constants.DriveConstants.wheelBaseLengthFeet, W = Constants.DriveConstants.wheelBaseWidthFeet;
		double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2) );

		frontRightX = strafe - rotate*(L/R);
		frontRightY = strafe + rotate*(L/R);
		backLeftX = forward - rotate*(W/R);
		backLeftY = forward + rotate*(W/R);

		wheelSpeed[0] = Math.sqrt((Math.pow(frontRightY,2) + Math.pow(backLeftX,2)));
		wheelSpeed[1] = Math.sqrt((Math.pow(frontRightY,2) + Math.pow(backLeftY,2)));
		wheelSpeed[2] = Math.sqrt((Math.pow(frontRightX,2) + Math.pow(backLeftY,2)));
		wheelSpeed[3] = Math.sqrt((Math.pow(frontRightX,2) + Math.pow(backLeftX,2)));
		normalizeSpeeds();

		wheelAngle[0] = Math.atan2(frontRightY, backLeftX) * 180/Math.PI;
		wheelAngle[1] = Math.atan2(frontRightY, backLeftY) * 180/Math.PI;
		wheelAngle[2] = Math.atan2(frontRightX, backLeftY) * 180/Math.PI;
		wheelAngle[3] = Math.atan2(frontRightX, backLeftX) * 180/Math.PI;

		for (int i = 0; i < wheelAngleChange.length; i++) {
			wheelAngleChange[i] = boundDegrees(wheelAngle[i] - currentAngle[i]);
		}

		if (!useAngleToReverse) {
			reverseWithSpeed(currentAngle);
		}
	}

	/**
	 * The kinematics create backwards vectors by rotating the wheels and always driving in a positive direction,
	 * (So going backwards would be Speed == 1.0, Angle == 180) </br>
	 * 
	 * If the vehicle design is such that it would better to avoid rotating the modules,
	 * use this method after the kinematics are done.
	 * (Then going backwards will be Speed ==-1.0, Angle == 0)
	 */
	private static void reverseWithSpeed(double[] currentAngle){
		for (int i = 0; i < wheelAngle.length; i++) {
			if (wheelAngleChange[i] > 90.0 || wheelAngleChange[i] < -90.0){
				wheelAngle[i] = wheelAngle[i] - Math.copySign(180.0, wheelAngle[i]);
				wheelAngleChange[i] = wheelAngleChange[i] - Math.copySign(180.0, wheelAngleChange[i]);
				wheelSpeed[i] *= -1.0;
			}
		}
	}

	/**Normalize speeds to be in a range from 0 to +1.0*/
	private static void normalizeSpeeds(){
		double max = Math.max(wheelSpeed[0], wheelSpeed[1]);
		max = Math.max(max, wheelSpeed[2]);
		max = Math.max(max, wheelSpeed[3]);

		if(max>1){
			wheelSpeed[0] /= max;
			wheelSpeed[1] /= max;
			wheelSpeed[2] /= max;
			wheelSpeed[3] /= max;
		}
	}

	// used by anti-tipping logic to avoid falling over our wheels
	public static void noSteering() {
		for (int i = 0; i < wheelAngleChange.length; i++) {
			wheelAngleChange[i] = 0;
		}
	}

	public static void setToFieldCentric(){
		fieldCentric = true;
		robotCentricOffsetRadians = 0;
	}

	public static void setToBotCentric(double offsetDeg){
		fieldCentric = false;
		robotCentricOffsetRadians = Math.toRadians(offsetDeg);
	}

	
	/**
	 * Reversing with Rotation means that when you go backwards,
	 * the speed will be in a positive direction but the angle will spin.
	 * This means that the speed is never negative </br>
	 * (So going backwards would be Speed == 1.0 , Angle == 180)
	 */
	public static void setReversingToRotation(){
		useAngleToReverse = true;
	}

	/**
	 * Reversing with Speed means that when you go backwards,
	 * the speed will be in a negative direction and there will be less rotation.
	 * This means that the angle is never outside the range -90 deg to 90 deg </br>
	 * (So going backwards would be Speed == -1.0 , Angle == 0)
	 */
	public static void setReversingToSpeed(){
		useAngleToReverse = false;
	}

	public static void setGyro(AHRS gyro){
		m_gyro = gyro;
		fieldCentric = true;
	}

	public static double getGyroYawDeg(){
		if (m_gyro != null && m_gyro.isConnected() && !m_gyro.isCalibrating()) {
			return m_gyro.getAngle();
		}
		else {
			return 0;
		}
	}

	// convert angle to range of +/- 180 degrees
	public static double boundDegrees(double angleDegrees) {
		double x = ((angleDegrees + 180) % 360) - 180;
		if (x < -180) {
			x += 360;
		}
		return x;
	}
}

