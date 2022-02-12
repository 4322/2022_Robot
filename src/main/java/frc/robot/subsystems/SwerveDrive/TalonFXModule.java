package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

public class TalonFXModule extends ControlModule {
	
	private WPI_TalonFX m_rotation; 
	private WPI_TalonFX m_wheel;
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos, int encoderID) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
		configDrive(wheel);
		configRotation(rotation, encoderID);
	}
//Need to configure both the drive and rotation motor controllers using the following example:
    private void configDrive(WPI_TalonFX talon) {
       
        TalonFXConfiguration config = new TalonFXConfiguration();
		config.openloopRamp = Constants.DriveConstants.Drive.configOpenLoopRamp;

		talon.configFactoryDefault();
		talon.configAllSettings(config);

        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);
		
		talon.configVoltageCompSaturation(Constants.DriveConstants.Drive.configVoltageCompSaturation);
		talon.enableVoltageCompensation(Constants.DriveConstants.Drive.enableVoltageCompensation);

		talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			Constants.DriveConstants.Drive.statorEnabled, 
			Constants.DriveConstants.Drive.statorLimit, 
			Constants.DriveConstants.Drive.statorThreshold, 
			Constants.DriveConstants.Drive.statorTime));
		talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			Constants.DriveConstants.Drive.supplyEnabled, 
			Constants.DriveConstants.Drive.supplyLimit, 
			Constants.DriveConstants.Drive.supplyThreshold, 
			Constants.DriveConstants.Drive.supplyTime));
    }

	private void configRotation(WPI_TalonFX talon, int encoderID) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.remoteFilter0.remoteSensorDeviceID = encoderID;
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        config.slot0.kP = Constants.DriveConstants.Rotation.kP;
        config.slot0.kD = Constants.DriveConstants.Rotation.kD;
		config.closedloopRamp = Constants.DriveConstants.Rotation.configCLosedLoopRamp;
        config.slot0.allowableClosedloopError = Constants.DriveConstants.Rotation.allowableClosedloopError;  
        config.motionAcceleration = Constants.DriveConstants.Rotation.motionAcceleration;
        config.motionCruiseVelocity = Constants.DriveConstants.Rotation.motionCruiseVelocity;

		talon.configFactoryDefault();
		talon.configAllSettings(config);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);
		
		talon.configVoltageCompSaturation(Constants.DriveConstants.Rotation.configVoltageCompSaturation);
		talon.enableVoltageCompensation(Constants.DriveConstants.Rotation.enableVoltageCompensation);
		
		talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			Constants.DriveConstants.Rotation.statorEnabled, 
			Constants.DriveConstants.Rotation.statorLimit, 
			Constants.DriveConstants.Rotation.statorThreshold, 
			Constants.DriveConstants.Rotation.statorTime));
		talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			Constants.DriveConstants.Rotation.supplyEnabled, 
			Constants.DriveConstants.Rotation.supplyLimit, 
			Constants.DriveConstants.Rotation.supplyThreshold, 
			Constants.DriveConstants.Rotation.supplyTime));
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
		return (m_rotation.getSelectedSensorPosition(0) *  Math.PI * Constants.DriveConstants.Drive.wheelDiameter);
	}
	
	@Override
	public double getDistance(){
		return (m_wheel.getSelectedSensorPosition(0) *  Math.PI * Constants.DriveConstants.Drive.wheelDiameter) / Constants.DriveConstants.Drive.ticksPerRev;
	}

	@Override
	public double getVelocity() {
		return m_wheel.getSelectedSensorVelocity(0) * 600/Constants.DriveConstants.Drive.ticksPerRev;
	}

	@Override
	public double getAcceleration() {
		return super.getAcceleration();
	}
}
