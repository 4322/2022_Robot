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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

// TO DO: switch from using PIDController to TalonFX PID

public class SwerveModule {
    
    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;

    private final PIDController m_drivePIDController = 
    new PIDController(Constants.DriveConstants.Drive.kP, Constants.DriveConstants.Drive.kI, Constants.DriveConstants.Drive.kD);

    private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          Constants.DriveConstants.Rotation.kP,
          0,
          Constants.DriveConstants.Rotation.kD,
          new TrapezoidProfile.Constraints(
              Constants.DriveConstants.kMaxAngularSpeed, Constants.DriveConstants.kModuleMaxAngularAcceleration));

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.DriveConstants.Drive.ks, Constants.DriveConstants.Drive.kv);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.DriveConstants.Rotation.ks, Constants.DriveConstants.Rotation.kv);
    
    public SwerveModule(
        int driveMotorChannel,
        int turningMotorChannel,
        int encoderID) {

        m_driveMotor = new TalonFX(driveMotorChannel); // distPerPulse: 2 * Math.PI * kWheelRadius / kEncoderResolution
        m_turningMotor = new TalonFX(turningMotorChannel); // distPerPulse: 2 * Math.PI / kEncoderResolution

        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
        return new SwerveModuleState(m_driveMotor.getSelectedSensorPosition(), new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

        // Calculate outputs
        final double driveOutput =
            m_drivePIDController.calculate(m_driveMotor.getSelectedSensorPosition(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput =
            m_turningPIDController.calculate(m_driveMotor.getSelectedSensorPosition(), state.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
        m_turningMotor.set(ControlMode.Current, turnOutput + turnFeedforward);
    }
}