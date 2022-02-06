package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class SwerveModule {
    
    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;
  
    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

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
    
    /**
    * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
    *
    * @param driveMotorChannel PWM output for the drive motor.
    * @param turningMotorChannel PWM output for the turning motor.
    * @param driveEncoderChannelA DIO input for the drive encoder channel A
    * @param driveEncoderChannelB DIO input for the drive encoder channel B
    * @param turningEncoderChannelA DIO input for the turning encoder channel A
    * @param turningEncoderChannelB DIO input for the turning encoder channel B
    */

    public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {

        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new TalonFX(turningMotorChannel);

        m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
        m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.kWheelRadius / Constants.DriveConstants.kEncoderResolution);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI / Constants.DriveConstants.kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

        // Calculate outputs
        final double driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turnOutput =
            m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
        m_turningMotor.set(ControlMode.Current, turnOutput + turnFeedforward);
    }
}