// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
    
    private final Translation2d m_frontLeftLocation = new Translation2d(Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
    private final Translation2d m_frontRightLocation = new Translation2d(Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
    private final Translation2d m_backLeftLocation = new Translation2d(-Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
    private final Translation2d m_backRightLocation = new Translation2d(-Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
  
    private final SwerveModule m_frontLeft = new SwerveModule(Constants.DriveConstants.frontRightDriveID, Constants.DriveConstants.frontRightRotationID);
    private final SwerveModule m_frontRight = new SwerveModule(Constants.DriveConstants.frontLeftDriveID, Constants.DriveConstants.frontLeftRotationID);
    private final SwerveModule m_backLeft = new SwerveModule(Constants.DriveConstants.backRightDriveID, Constants.DriveConstants.backRightRotationID);
    private final SwerveModule m_backRight = new SwerveModule(Constants.DriveConstants.backLeftDriveID, Constants.DriveConstants.backLeftRotationID);

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
