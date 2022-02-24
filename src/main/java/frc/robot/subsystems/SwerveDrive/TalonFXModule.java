package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class TalonFXModule extends ControlModule {
	
	private WPI_TalonFX m_rotation; 
	private WPI_TalonFX m_wheel;
	private CANCoder m_encoder;
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos, int encoderID) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
		configDrive(wheel);
		configRotation(rotation, encoderID);
	}

    private void configDrive(WPI_TalonFX talon) {
       
        TalonFXConfiguration config = new TalonFXConfiguration();
		config.openloopRamp = Constants.DriveConstants.Drive.configOpenLoopRamp;

		talon.configFactoryDefault();
		talon.configAllSettings(config);

        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
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
        config.slot0.kP = Constants.DriveConstants.Rotation.kP;
        config.slot0.kD = Constants.DriveConstants.Rotation.kD;
		config.closedloopRamp = Constants.DriveConstants.Rotation.configCLosedLoopRamp;
        config.slot0.allowableClosedloopError = Constants.DriveConstants.Rotation.allowableClosedloopError;  
		config.nominalOutputForward = Constants.DriveConstants.Rotation.minPower;
		config.nominalOutputReverse = -Constants.DriveConstants.Rotation.minPower;
		config.peakOutputForward = Constants.DriveConstants.Rotation.maxPower;
		config.peakOutputReverse = -Constants.DriveConstants.Rotation.maxPower;

		talon.configFactoryDefault();
		talon.configAllSettings(config);
        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
        talon.setInverted(false);
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

		CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
		encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
		encoderConfig.sensorDirection = true;  // positive rotation is clockwise
		encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		
		m_encoder = new CANCoder(encoderID);
		m_encoder.configFactoryDefault();
		m_encoder.configAllSettings(encoderConfig);

		// initialize internal Falcon encoder to absolute wheel position from CANCoder
		talon.setSelectedSensorPosition(m_encoder.getAbsolutePosition() / 
			Constants.DriveConstants.Rotation.countToDegrees);
    }
	
	public void setSpeedAndAngle(){
		m_wheel.set(ControlMode.PercentOutput, SwerveHelper.getSpeed(position));
		m_rotation.set(ControlMode.Position, getInternalRotationCount() + 
			SwerveHelper.getAngleChange(position) / Constants.DriveConstants.Rotation.countToDegrees);
	}	

	public void setRotationPID(double kp, double ki, double kd) {
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
	
	public double getMagneticRotationAngle(){
		return m_encoder.getAbsolutePosition();
	}

	public double getInternalRotationCount(){
		return m_rotation.getSelectedSensorPosition();
	}

	// returns +/- 180 degrees
	public double getInternalRotationDegrees(){
		return SwerveHelper.boundDegrees(getInternalRotationCount() * 
			   Constants.DriveConstants.Rotation.countToDegrees);
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
