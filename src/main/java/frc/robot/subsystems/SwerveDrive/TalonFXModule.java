package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonFXModule extends ControlModule {
	
	private WPI_TalonFX m_rotation; 
	private WPI_TalonFX m_wheel;
	
	private double wheelDiameter = 4.0;
	private double rotationDiameter = 4.0;
	private int ticksPerRev = 4096;
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
	}
	
	public void setSpeedAndAngle(Joystick drive, Joystick rotate){
		m_wheel.set(ControlMode.PercentOutput, SwerveHelper.getSpeedValue(drive, rotate, position.wheelNumber));
		m_rotation.set(ControlMode.Position, SwerveHelper.getAngleValue(drive, rotate, position.wheelNumber));
	}
	
	public void setRotationPID(double kp, double ki, double kd){
		m_rotation.config_kP(0, kp, 0);
		m_rotation.config_kI(1, ki, 0);
		m_rotation.config_kD(2, kd, 0);
	}

	public double getRotationP(){
		return m_rotation.configGetParameter(0, 0, 0);
	}

	public double getRotationI(){
		return m_rotation.configGetParameter(1, 0, 0);
	}

	public double getRotationD(){
		return m_rotation.configGetParameter(2, 0, 0);
	}
	
	@Override
	public double getAngle(){
		return (m_rotation.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter);
	}
	
	@Override
	public double getDistance(){
		return (m_wheel.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter) / ticksPerRev;
	}

	@Override
	public double getVelocity() {
		return m_wheel.getSelectedSensorVelocity(0) * 600/ticksPerRev;
	}

	@Override
	public double getAcceleration() {
		return super.getAcceleration();
	}

	public double getWheelDiameter() {
		return wheelDiameter;
	}

	public void setWheelDiameter(double wheelDiameter) {
		this.wheelDiameter = wheelDiameter;
	}

	public double getRotationDiameter() {
		return rotationDiameter;
	}

	public void setRotationDiameter(double rotationDiameter) {
		this.rotationDiameter = rotationDiameter;
	}

	public int getTicksPerRev() {
		return ticksPerRev;
	}

	public void setTicksPerRev(int ticksPerRev) {
		this.ticksPerRev = ticksPerRev;
	}	

}
