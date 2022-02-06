// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.SwerveDrive;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;

public class Drivebase {
    
    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
  
    private final SwerveModule m_frontLeft; // new SwerveModule(1, 2, 0, 1, 2, 3);
    private final SwerveModule m_frontRight; // new SwerveModule(3, 4, 4, 5, 6, 7);
    private final SwerveModule m_backLeft; // new SwerveModule(5, 6, 8, 9, 10, 11);
    private final SwerveModule m_backRight; // new SwerveModule(7, 8, 12, 13, 14, 15);

    // need to initialize swerve module later

    private final AHRS m_gyro = new AHRS();

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    public Drivebase() {
        m_gyro.reset();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) { // drive w/ joystick
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState());
      }

}
