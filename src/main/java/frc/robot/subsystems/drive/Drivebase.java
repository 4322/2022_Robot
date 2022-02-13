// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SerialPort;
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
          
    private WPI_TalonFX frontRightDrive;
    private WPI_TalonFX frontLeftDrive;
    private WPI_TalonFX rearRightDrive;
    private WPI_TalonFX rearLeftDrive;
    
    private WPI_TalonFX frontRightRotation;
    private WPI_TalonFX frontLeftRotation;
    private WPI_TalonFX rearRightRotation;
    private WPI_TalonFX rearLeftRotation;

    private SwerveModule frontRight;
    private SwerveModule frontLeft;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;
    
    public Drivebase() {
        if (Constants.driveEnabled) {
            frontRight = new SwerveModule(
                Constants.DriveConstants.frontRightDriveID,
                Constants.DriveConstants.frontRightRotationID,
                Constants.DriveConstants.frontRightEncoderID);
            frontLeft = new SwerveModule(
                Constants.DriveConstants.frontLeftDriveID,
                Constants.DriveConstants.frontLeftRotationID,
                Constants.DriveConstants.frontLeftEncoderID);
            rearLeft = new SwerveModule(
                Constants.DriveConstants.backRightDriveID,
                Constants.DriveConstants.backRightRotationID,
                Constants.DriveConstants.backRightEncoderID);
            rearRight = new SwerveModule(
                Constants.DriveConstants.backLeftDriveID,
                Constants.DriveConstants.backLeftRotationID,
                Constants.DriveConstants.backLeftEncoderID); 
        }
    }
    
    // need to initialize swerve module later
    private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1); 

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) { // drive w/ joystick
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState());
        }    

    public void setCoastMode() {
        if (Constants.driveEnabled) {
            frontRightDrive.setNeutralMode(NeutralMode.Coast);
            frontLeftDrive.setNeutralMode(NeutralMode.Coast);
            rearRightDrive.setNeutralMode(NeutralMode.Coast);
            rearLeftDrive.setNeutralMode(NeutralMode.Coast);
            frontRightRotation.setNeutralMode(NeutralMode.Coast);
            frontLeftRotation.setNeutralMode(NeutralMode.Coast);
            rearRightRotation.setNeutralMode(NeutralMode.Coast);
            rearLeftRotation.setNeutralMode(NeutralMode.Coast);
            }
    }

    public void setBrakeMode() {
        if (Constants.driveEnabled) {
            frontRightDrive.setNeutralMode(NeutralMode.Brake); 
            frontLeftDrive.setNeutralMode(NeutralMode.Brake);
            rearRightDrive.setNeutralMode(NeutralMode.Brake);
            rearLeftDrive.setNeutralMode(NeutralMode.Brake);
            frontRightRotation.setNeutralMode(NeutralMode.Brake);
            frontLeftRotation.setNeutralMode(NeutralMode.Brake);
            rearRightRotation.setNeutralMode(NeutralMode.Brake);
            rearLeftRotation.setNeutralMode(NeutralMode.Brake);
        }
    }
}
