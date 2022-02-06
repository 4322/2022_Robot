// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/SwerveModule.java
package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.DriveConstants.kMaxAngularSpeed, Constants.DriveConstants.kModuleMaxAngularAcceleration));

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
    
    public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {

        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new TalonFX(turningMotorChannel);

        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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