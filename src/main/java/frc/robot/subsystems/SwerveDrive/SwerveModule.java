package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class SwerveModule extends ControlModule {
	
	private WPI_TalonFX turningMotor; 
	private WPI_TalonFX driveMotor;
	private CANCoder encoder;
	private WheelPosition wheelPosition;
	
	public SwerveModule(int rotationID, int wheelID, WheelPosition pos, int encoderID) {
		super(pos);
		turningMotor = new WPI_TalonFX(rotationID);
		driveMotor = new WPI_TalonFX(wheelID);
		encoder = new CANCoder(encoderID);
		wheelPosition = pos;

		RobotContainer.staggerTalonStatusFrames(driveMotor);
		RobotContainer.staggerTalonStatusFrames(turningMotor);
		encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
	}

	public void init() {
		configDrive(driveMotor, wheelPosition);
		configRotation(turningMotor);
	}

    private void configDrive(WPI_TalonFX talon, WheelPosition pos) {
       
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = DriveConstants.Drive.kP;
    config.slot0.kI = DriveConstants.Drive.kI;
    config.slot0.kD = DriveConstants.Drive.kD;
    config.slot0.integralZone = DriveConstants.Drive.kIz;
    config.slot0.kF = DriveConstants.Drive.kFF;
		config.closedloopRamp = DriveConstants.Drive.configClosedLoopRamp;
		config.neutralDeadband = DriveConstants.Drive.brakeModeDeadband; // delay brake mode activation for tipping

		talon.configAllSettings(config);

    talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
		boolean isRightSide = pos == WheelPosition.FRONT_RIGHT || pos == WheelPosition.BACK_RIGHT;
    talon.setInverted(!isRightSide);
    talon.setSensorPhase(false);
		
		talon.configVoltageCompSaturation(DriveConstants.Drive.voltageCompSaturation);
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
    talon.setInverted(true);
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
		encoderConfig.sensorDirection = false;  // positive rotation is CCW
		encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		
		encoder.configAllSettings(encoderConfig);  // factory default is the baseline

		// need fast initial reading from the CANCoder
		encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			10, Constants.controllerConfigTimeoutMs);

		try {
			Thread.sleep(50); // 5 status frames to be safe
		}
		catch (InterruptedException e) {}

		// initialize internal Falcon encoder to absolute wheel position from CANCoder
		double count = 	(encoder.getAbsolutePosition() - 
                     DriveConstants.Rotation.CANCoderOffsetDegrees[position.wheelNumber]) / 
                       DriveConstants.Rotation.countToDegrees;

		ErrorCode error = talon.setSelectedSensorPosition(count, 0, Constants.controllerConfigTimeoutMs);
		if (error != ErrorCode.OK) {
			DriverStation.reportError("Error " + error.value + " initializing Talon FX " + talon.getDeviceID() + 
				" position ", false);
		}

		// don't need the CANCoder any longer, so a slow frame rate is OK
		encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 
			RobotContainer.nextSlowStatusPeriodMs(),
			Constants.controllerConfigTimeoutMs);

		// need rapid position feedback for steering logic
		turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
			RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    }
	
	public double getMagneticRotationAngle(){
		return encoder.getAbsolutePosition();
	}

	public double getInternalRotationCount(){
		return turningMotor.getSelectedSensorPosition();
	}

	// returns +/- 180 degrees
	public double getInternalRotationDegrees(){
		return Drive.boundDegrees(getInternalRotationCount() * 
			   DriveConstants.Rotation.countToDegrees);
	}

	@Override
	public double getDistance(){
		return driveMotor.getSelectedSensorPosition(0) / DriveConstants.encoderResolution /
			Constants.DriveConstants.Drive.gearRatio * Math.PI *
			DriveConstants.Drive.wheelDiameterInches / 12;
	}

	@Override
	public double getVelocity() {
		// feet per second
		return driveMotor.getSelectedSensorVelocity(0) * 10 / DriveConstants.encoderResolution /
			Constants.DriveConstants.Drive.gearRatio * Math.PI * 
			Constants.DriveConstants.Drive.wheelDiameterInches / 12;
	}

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), 
        Rotation2d.fromDegrees(
            turningMotor.getSelectedSensorPosition() * DriveConstants.Rotation.countToDegrees));
}

public void setDesiredState(SwerveModuleState desiredState) {
	double currentDeg = turningMotor.getSelectedSensorPosition() * DriveConstants.Rotation.countToDegrees;

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(currentDeg));

    driveMotor.set(ControlMode.Velocity, 
        state.speedMetersPerSecond / 
        (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
        * DriveConstants.Drive.gearRatio * DriveConstants.encoderResolution
        / 10); // every 100 ms

	// Calculate the change in degrees and add that to the current position
    turningMotor.set(ControlMode.Position, 
        (currentDeg + Drive.boundDegrees(state.angle.getDegrees() - currentDeg))
		/ DriveConstants.Rotation.countToDegrees);
}

public void setCoastMode() {
	driveMotor.setNeutralMode(NeutralMode.Coast);
	turningMotor.setNeutralMode(NeutralMode.Coast);
}

public void setBrakeMode() {
	driveMotor.setNeutralMode(NeutralMode.Brake);
	turningMotor.setNeutralMode(NeutralMode.Brake);
}

public void stop() {
	driveMotor.stopMotor();
	turningMotor.stopMotor();
}
}
