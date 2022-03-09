package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;
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
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos, int encoderID) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
		configDrive(wheel, pos);
		configRotation(rotation, encoderID);

		// increase status reporting periods to reduce CAN bus utilization
		m_wheel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
			Constants.slowControllerStatusPeriodMs, Constants.controllerConfigTimeoutMs);
		m_wheel.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
			Constants.slowControllerStatusPeriodMs, Constants.controllerConfigTimeoutMs);
		m_rotation.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
			Constants.slowControllerStatusPeriodMs, Constants.controllerConfigTimeoutMs);

		// don't need the CANCoder any longer, so a slow frame rate is OK
		m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			DriveConstants.Rotation.CANCoderStatusFramePeriodMs, 
			Constants.controllerConfigTimeoutMs);
	}

    private void configDrive(WPI_TalonFX talon, WheelPosition pos) {
       
        TalonFXConfiguration config = new TalonFXConfiguration();
		config.openloopRamp = DriveConstants.Drive.configOpenLoopRamp;

		talon.configFactoryDefault();
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
    }

	private void configRotation(WPI_TalonFX talon, int encoderID) {

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
		
		m_encoder = new CANCoder(encoderID);
		m_encoder.configAllSettings(encoderConfig);  // factory default is the baseline

		// wait for CANCoder position to stabilize
		try {
			Thread.sleep(50); // can go as low as 10, 50 to be safe
		}
		catch (InterruptedException e) {}

		// initialize internal Falcon encoder to absolute wheel position from CANCoder
		double count = 	(m_encoder.getAbsolutePosition() - 
		DriveConstants.Rotation.CANCoderOffsetDegrees[position.wheelNumber]) / 
	   	DriveConstants.Rotation.countToDegrees;

		// don't need the CANCoder any longer, so increase reporting period to
		// reduce CAN bus utilization
		m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 500, 100);

		ErrorCode error = talon.setSelectedSensorPosition(count, 0, Constants.controllerConfigTimeoutMs);
		if (error != ErrorCode.OK) {
			DriverStation.reportError("Error " + error.value + " initializing Talon FX " + talon.getDeviceID() + 
				" position ", false);
		}
    }
	
	public void setSpeedAndAngle(){
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

	private int unique = 0;

	public void testCAN() {
		//m_wheel.setSelectedSensorPosition(unique++, 0, 10);
		m_rotation.setSelectedSensorPosition(unique++, 0, 10);
	}
}
 