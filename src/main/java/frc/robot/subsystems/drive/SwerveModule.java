// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/SwerveModule.java
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private CANCoder m_encoder;

    public SwerveModule(
        int driveMotorChannel,
        int turningMotorChannel,
        int encoderID) {

        driveMotor = new TalonFX(driveMotorChannel);
        turningMotor = new TalonFX(turningMotorChannel);
        
        configDrive(driveMotor);
        configRotation(turningMotor, encoderID);
    }
    
    private void configDrive(TalonFX talon) {
       
        TalonFXConfiguration config = new TalonFXConfiguration();
		config.closedloopRamp = DriveConstants.Drive.rampRate;
        config.slot0.kP = DriveConstants.Drive.kP;
        config.slot0.kI = DriveConstants.Drive.kI;
        config.slot0.kD = DriveConstants.Drive.kD;
        config.slot0.allowableClosedloopError = DriveConstants.Drive.allowableClosedloopError;  

		talon.configFactoryDefault();
		talon.configAllSettings(config);

        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
        talon.setInverted(true);
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

	private void configRotation(TalonFX talon, int encoderID) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = DriveConstants.Rotation.kP;
        config.slot0.kD = DriveConstants.Rotation.kD;
		config.closedloopRamp = DriveConstants.Rotation.configCLosedLoopRamp;
        config.slot0.allowableClosedloopError = DriveConstants.Rotation.allowableClosedloopError;  

		talon.configFactoryDefault();
		talon.configAllSettings(config);
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
        encoderConfig.sensorDirection = true;  // positive rotation is clockwise
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        
        m_encoder = new CANCoder(encoderID);
        m_encoder.configFactoryDefault();
        m_encoder.configAllSettings(encoderConfig);

        // initialize internal Falcon encoder to absolute wheel position from CANCoder
        talon.setSelectedSensorPosition(m_encoder.getAbsolutePosition() / 
            DriveConstants.Rotation.countToDegrees);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), 
            Rotation2d.fromDegrees(
                turningMotor.getSelectedSensorPosition() * DriveConstants.Rotation.countToDegrees));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(
                turningMotor.getSelectedSensorPosition() * DriveConstants.Rotation.countToDegrees));

        driveMotor.set(ControlMode.Velocity, 
            state.speedMetersPerSecond / 
            (DriveConstants.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
            * DriveConstants.driveGearRatio * DriveConstants.encoderResolution
            / 10); // every 100 ms
        turningMotor.set(ControlMode.Position, 
            state.angle.getDegrees() / DriveConstants.Rotation.countToDegrees);
    }
}