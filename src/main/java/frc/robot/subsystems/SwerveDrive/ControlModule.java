package frc.robot.subsystems.SwerveDrive;


import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A ControlModule allows you to easily control the speed controllers for 
 * both rotation and speed</br>
 * The rotation speed controller has a PIDController with an potentiometer source. 
 * <i>(TODO: setup an option to use absolute encoders in place of rotation potentiometers)</i></br>
 * <i>(TODO:Create a TalonSRXModule based on this)</i></br>
 * 
 * @author created by Unnas Hussain on 8/2/2017
 */
public class ControlModule{

	public WheelPosition position;

	private PWMMotorController m_rotation; 
	private PWMMotorController m_wheel; 
	//NEW INSTANCE VARIABLES: continuousRotationStart, continuousRotationEnd
	private double continuousRotationStart; //values not yet declared; to be determined later
	private double continuousRotationEnd; //values not yet declared; to be determined later
	private PIDController rotationPID;
	
	public double rotateP = 1.0;
	public double rotateI = 0.0;
	public double rotateD = 0.0;
	protected double rotatePIDTolerance = 0.01;

	private AnalogPotentiometer rotationEncoder = null;
	private Encoder speedEncoder = null;

	protected double previousRate = 0;
	protected double previousTime = 0;
	protected double filteredAccel = 0;
	
	protected ControlModule(WheelPosition pos) {
		position = pos;
	}

	public ControlModule(PWMMotorController rotation, PWMMotorController wheel, int potentiometerPort, WheelPosition pos){
		position = pos;

		m_rotation = rotation;
		m_wheel = wheel;
		rotationEncoder = new AnalogPotentiometer(potentiometerPort, 360.0, 0);

		rotationPID = new PIDController(rotateP, rotateI, rotateD, rotationEncoder, m_rotation);

		rotationPID.enableContinuousInput(continuousRotationStart,continuousRotationEnd); //changed setContinuous to enableContinuousInput and added new instance variables
		rotationPID.setTolerance(rotatePIDTolerance);
	}

	public ControlModule(PWMMotorController rotation, PWMMotorController wheel, int encoderPort, int wheelEncoderA, int wheelEncoderB, WheelPosition location){
		this(rotation, wheel, encoderPort, location);
		speedEncoder = new Encoder(wheelEncoderA, wheelEncoderB);
	}

	public void setSpeedAndAngle(Joystick drive, Joystick rotate){
		m_wheel.set(SwerveHelper.getSpeedValue(drive, rotate, position.wheelNumber));
		rotationPID.setSetpoint(SwerveHelper.getAngleValue(drive, rotate, position.wheelNumber));
	}

	public void setEncoder(int encoderPortA, int encoderPortB){
		this.speedEncoder = new Encoder(encoderPortA, encoderPortB);
	}

	//*********************PID Helper Methods******************//

	public void setRotationPIDTolerance(double tolerance) {
		rotatePIDTolerance = tolerance;
		rotationPID.setTolerance(tolerance);
	}
	
	public void setRotationPID(double kp, double ki, double kd){
		rotationPID.setPID(kp, ki, kd);
	}

	public void setRotationPIDF(double kp, double ki, double kd, double kf){
		rotationPID.setPID(kp, ki, kd, kf); //No method with kf
	}

	public double getRotationP(){
		return rotationPID.getP();
	}

	public double getRotationI(){
		return rotationPID.getI();
	}

	public double getRotationD(){
		return rotationPID.getD();
	}

	public double getRotationF(){
		return rotationPID.getF();
	}

	public void resetSpeedEncoder(){
		if(speedEncoder != null)
			this.speedEncoder.reset();
	}

	//************** Pos/Vel/Acc Helper Methods **************// 

	public double getAngle(){
		if(rotationEncoder != null)
			return rotationEncoder.get();
		return -1.0;
	}

	public double getDistance() {
		if(speedEncoder != null)
			return speedEncoder.getDistance();
		return 0.0;
	}

	public double getVelocity() {
		if(speedEncoder != null)
			return speedEncoder.getRate();
		return 0.0;

	}

	public double getAcceleration() {

		double currentRate = this.getVelocity();
		double currentTime = Timer.getFPGATimestamp();

		double acceleration = (currentRate - previousRate) / (currentTime - previousTime);

		previousRate = currentRate;
		previousTime = currentTime;

		filteredAccel = acceleration * 0.5 + filteredAccel * 0.5; //dampens random spikes due to the fact that we are deriving this value

		return filteredAccel;
	}	

	public Encoder getSpeedEncoder(){
		if(speedEncoder != null) 
			return this.speedEncoder;
		return null;
	}

	public AnalogPotentiometer getRotationEncoder() {
		if(rotationEncoder != null)
			return this.rotationEncoder;
		return null;
	}
	
	public void debug() {
		String name = position.toString() + " ControlModule/";
		
		SmartDashboard.putNumber(name + "Distance", this.getDistance());
		SmartDashboard.putNumber(name + "Velocity", this.getVelocity());
		SmartDashboard.putNumber(name + "Acceleration", this.getAcceleration());
		
		SmartDashboard.putNumber(name + "Spin P", this.getRotationP());
		SmartDashboard.putNumber(name + "Spin I", this.getRotationI());
		SmartDashboard.putNumber(name + "Spin D", this.getRotationD());
		SmartDashboard.putNumber(name + "Spin F", this.getRotationF());
	}

	public enum WheelPosition{
		FRONT_RIGHT(0), FRONT_LEFT(1), BACK_LEFT(2), BACK_RIGHT(3);

		public int wheelNumber;

		WheelPosition(int id){
			wheelNumber = id;
		}

		public String toString() {
			switch(wheelNumber) {
			case 0: return "FRONT_RIGHT";
			case 1: return "FRONT_LEFT";
			case 2: return "BACK_LEFT";
			case 3: return "BACK_RIGHT";
			default: return "???";
			}
		}
	}
}
