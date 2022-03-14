package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class TalonFXModule extends ControlModule {
	
	private WPI_TalonFX m_rotation; 
	private WPI_TalonFX m_wheel;
	private CANCoder m_encoder;
	private WheelPosition m_wheelPosition;
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos, int encoderID) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
		m_encoder = new CANCoder(encoderID);
		m_wheelPosition = pos;

		RobotContainer.staggerTalonStatusFrames(m_wheel);
		RobotContainer.staggerTalonStatusFrames(m_rotation);
		m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
	}

	public void init() {
		configDrive(m_wheel, m_wheelPosition);
		configRotation(m_rotation);
	}

    private void configDrive(WPI_TalonFX talon, WheelPosition pos) {
       
        TalonFXConfiguration config = new TalonFXConfiguration();
		config.openloopRamp = DriveConstants.Drive.configOpenLoopRamp;
		config.neutralDeadband = DriveConstants.Drive.brakeModeDeadband; // delay brake mode activation for tipping

		talon.configAllSettings(config);

        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
		boolean isRightSide = pos == WheelPosition.FRONT_RIGHT || pos == WheelPosition.BACK_RIGHT;
        talon.setInverted(isRightSide);
        talon.setSensorPhase(false);
		
		talon.configVoltageCompSaturation(DriveConstants.Drive.configVoltageCompSaturation);
		talon.enableVoltageCompensation(DriveConstants.Drive.enableVoltageCompensation);

		talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			DriveConstants.Drive.statorEnabled, 
			DriveConstants.Drive.statorLimit, 
			DriveConstants.Drive.statorThreshold, 
			DriveConstants.Drive.statorTime));
		talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			DriveConstants.Drive.supplyEnabled, 
			DriveConstants.Drive.supplyLimit, 
			DriveConstants.Drive.supplyThreshold, 
			DriveConstants.Drive.supplyTime));

		// need rapid velocity feedback for anti-tipping logic
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
			RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    }

	private void configRotation(WPI_TalonFX talon) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = DriveConstants.Rotation.kP;
        config.slot0.kD = DriveConstants.Rotation.kD;
		config.closedloopRamp = DriveConstants.Rotation.configCLosedLoopRamp;
        config.slot0.allowableClosedloopError = DriveConstants.Rotation.allowableClosedloopError;  
		config.nominalOutputForward = DriveConstants.Rotation.minPower;
		config.nominalOutputReverse = -DriveConstants.Rotation.minPower;
		config.peakOutputForward = DriveConstants.Rotation.maxPower;
		config.peakOutputReverse = -DriveConstants.Rotation.maxPower;

		talon.configAllSettings(config);	// factory default is the baseline
        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
        talon.setInverted(false);
        talon.setSensorPhase(false);
		
		talon.configVoltageCompSaturation(DriveConstants.Rotation.configVoltageCompSaturation);
		talon.enableVoltageCompensation(DriveConstants.Rotation.enableVoltageCompensation);
		
		talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			DriveConstants.Rotation.statorEnabled, 
			DriveConstants.Rotation.statorLimit, 
			DriveConstants.Rotation.statorThreshold, 
			DriveConstants.Rotation.statorTime));
		talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			DriveConstants.Rotation.supplyEnabled, 
			DriveConstants.Rotation.supplyLimit, 
			DriveConstants.Rotation.supplyThreshold, 
			DriveConstants.Rotation.supplyTime));

		CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
		encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
		encoderConfig.sensorDirection = true;  // positive rotation is clockwise
		encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		
		m_encoder.configAllSettings(encoderConfig);  // factory default is the baseline

		// need fast initial reading from the CANCoder
		m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			10, Constants.controllerConfigTimeoutMs);

		try {
			Thread.sleep(50); // 5 status frames to be safe
		}
		catch (InterruptedException e) {}

		// initialize internal Falcon encoder to absolute wheel position from CANCoder
		double count = 	(m_encoder.getAbsolutePosition() - 
			DriveConstants.Rotation.CANCoderOffsetDegrees[position.wheelNumber]) / 
			DriveConstants.Rotation.countToDegrees;

		ErrorCode error = talon.setSelectedSensorPosition(count, 0, Constants.controllerConfigTimeoutMs);
		if (error != ErrorCode.OK) {
			DriverStation.reportError("Error " + error.value + " initializing Talon FX " + talon.getDeviceID() + 
				" position ", false);
		}

		// don't need the CANCoder any longer, so a slow frame rate is OK
		m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			RobotContainer.nextSlowStatusPeriodMs(),
			Constants.controllerConfigTimeoutMs);

		// need rapid position feedback for steering logic
		m_rotation.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
			RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    }
	
	public void setSpeedAndAngle(){
		m_wheel.set(ControlMode.PercentOutput, SwerveHelper.getSpeed(position));
		m_rotation.set(ControlMode.Position, getInternalRotationCount() + 
			SwerveHelper.getAngleChange(position) / 
			DriveConstants.Rotation.countToDegrees);
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
			   DriveConstants.Rotation.countToDegrees);
	}

	@Override
	public double getDistance(){
		return (m_wheel.getSelectedSensorPosition(0) *  Math.PI * DriveConstants.Drive.wheelDiameter) / 
			DriveConstants.Drive.ticksPerRev;
	}

	@Override
	public double getVelocity() {
		return m_wheel.getSelectedSensorVelocity(0) * 600/DriveConstants.Drive.ticksPerRev;
	}

	@Override
	public double getAcceleration() {
		return super.getAcceleration();
	}

	public void stop() {
		m_wheel.stopMotor();
		m_rotation.stopMotor();
	}
}
