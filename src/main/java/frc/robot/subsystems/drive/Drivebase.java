// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
import frc.robot.subsystems.SwerveDrive.TalonFXModule;

public class Drivebase extends SubsystemBase {
    private WPI_TalonFX frontRightDrive;
    private WPI_TalonFX frontLeftDrive;
    private WPI_TalonFX rearRightDrive;
    private WPI_TalonFX rearLeftDrive;
    
    private WPI_TalonFX frontRightRotation;
    private WPI_TalonFX frontLeftRotation;
    private WPI_TalonFX rearRightRotation;
    private WPI_TalonFX rearLeftRotation;

    private TalonFXModule frontRight;
    private TalonFXModule frontLeft;
    private TalonFXModule rearLeft;
    private TalonFXModule rearRight;
    
    public void Drivebase() {
        if (Constants.driveEnabled) {
            private final Translation2d m_frontLeftLocation = new Translation2d(Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
            private final Translation2d m_frontRightLocation = new Translation2d(Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
            private final Translation2d m_backLeftLocation = new Translation2d(-Constants.DriveConstants.distWheelX, Constants.DriveConstants.distWheelY);
            private final Translation2d m_backRightLocation = new Translation2d(-Constants.DriveConstants.distWheelX, -Constants.DriveConstants.distWheelY);
          
            private final SwerveModule m_frontLeft = new SwerveModule(
                Constants.DriveConstants.frontRightDriveID,
                Constants.DriveConstants.frontRightRotationID,
                Constants.DriveConstants.frontRightEncoderID);
            private final SwerveModule m_frontRight = new SwerveModule(
                Constants.DriveConstants.frontLeftDriveID,
                Constants.DriveConstants.frontLeftRotationID,
                Constants.DriveConstants.frontLeftEncoderID);
            private final SwerveModule m_backLeft = new SwerveModule(
                Constants.DriveConstants.backRightDriveID,
                Constants.DriveConstants.backRightRotationID,
                Constants.DriveConstants.backRightEncoderID);
            private final SwerveModule m_backRight = new SwerveModule(
                Constants.DriveConstants.backLeftDriveID,
                Constants.DriveConstants.backLeftRotationID,
                Constants.DriveConstants.backLeftEncoderID); 

            if (Constants.gyroEnabled) {
                SwerveHelper.setGyro(new AHRS(SerialPort.Port.kUSB1));
            }
        }
}
        
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
}
