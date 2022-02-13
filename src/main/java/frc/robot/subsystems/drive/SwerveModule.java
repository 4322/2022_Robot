// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/SwerveModule.java
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

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
		config.closedloopRamp = Constants.DriveConstants.Drive.rampRate;
        config.slot0.kP = Constants.DriveConstants.Drive.kP;
        config.slot0.kI = Constants.DriveConstants.Drive.kI;
        config.slot0.kD = Constants.DriveConstants.Drive.kD;
        config.slot0.allowableClosedloopError = Constants.DriveConstants.Drive.allowableClosedloopError;  

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

	private void configRotation(TalonFX talon, int encoderID) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.remoteFilter0.remoteSensorDeviceID = encoderID;
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        config.slot0.kP = Constants.DriveConstants.Rotation.kP;
        config.slot0.kD = Constants.DriveConstants.Rotation.kD;
		config.closedloopRamp = Constants.DriveConstants.Rotation.configCLosedLoopRamp;
        config.slot0.allowableClosedloopError = Constants.DriveConstants.Rotation.allowableClosedloopError;  

		talon.configFactoryDefault();
		talon.configAllSettings(config);
        talon.setNeutralMode(NeutralMode.Coast); //Allow robot to be moved prior to enabling
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), new Rotation2d(turningMotor.getSelectedSensorPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getSelectedSensorPosition()));

        driveMotor.set(ControlMode.Velocity, 
            state.speedMetersPerSecond / 
            (Constants.DriveConstants.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
            * Constants.DriveConstants.driveGearRatio * Constants.DriveConstants.kEncoderResolution
            / 10); // every 100 ms
        turningMotor.set(ControlMode.Position, state.angle.getDegrees());
    }
}