package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
import frc.robot.subsystems.SwerveDrive.TalonFXModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

public class Driveunbun extends SubsystemBase {
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

    public Driveunbun() {
        if (Constants.driveEnabled) {
            frontRightDrive = new WPI_TalonFX(Constants.DriveConstants.frontRightDriveID);
            frontLeftDrive = new WPI_TalonFX(Constants.DriveConstants.frontLeftDriveID);
            rearRightDrive = new WPI_TalonFX(Constants.DriveConstants.rearRightDriveID);
            rearLeftDrive = new WPI_TalonFX(Constants.DriveConstants.rearLeftDriveID);
            frontRightRotation = new WPI_TalonFX(Constants.DriveConstants.frontRightRotationID);
            frontLeftRotation = new WPI_TalonFX(Constants.DriveConstants.frontLeftRotationID);
            rearRightRotation = new WPI_TalonFX(Constants.DriveConstants.rearRightRotationID);
            rearLeftRotation = new WPI_TalonFX(Constants.DriveConstants.rearLeftRotationID);
            
            frontRight = new TalonFXModule(frontRightRotation, frontRightDrive, 
                WheelPosition.FRONT_RIGHT, Constants.DriveConstants.frontRightEncoderID);
            frontLeft = new TalonFXModule(frontLeftRotation, frontLeftDrive, 
                WheelPosition.FRONT_LEFT, Constants.DriveConstants.frontLeftEncoderID);
            rearRight = new TalonFXModule(rearRightRotation, rearRightDrive, 
                WheelPosition.BACK_RIGHT, Constants.DriveConstants.rearRightEncoderID); 
            rearLeft = new TalonFXModule(rearLeftRotation, rearLeftDrive, 
                WheelPosition.BACK_LEFT, Constants.DriveConstants.rearLeftEncoderID);

            if (Constants.gyroEnabled) {
                SwerveHelper.setGyro(new AHRS(SerialPort.Port.kUSB1));
            }

            SwerveHelper.setReversingToSpeed();
        }   
    }

    public void setSpeedAndAngle(Joystick drive, Joystick rotate){
        if (Constants.driveEnabled) {
            double[] currentAngle = new double[4];
            currentAngle[WheelPosition.FRONT_RIGHT.wheelNumber] = frontRight.getInternalRotationDegrees();
            currentAngle[WheelPosition.FRONT_LEFT.wheelNumber] = frontLeft.getInternalRotationDegrees();
            currentAngle[WheelPosition.BACK_RIGHT.wheelNumber] = rearRight.getInternalRotationDegrees();
            currentAngle[WheelPosition.BACK_LEFT.wheelNumber] = rearLeft.getInternalRotationDegrees();

            SwerveHelper.calculate(drive.getY(), drive.getX(), rotate.getX(), currentAngle);
            frontRight.setSpeedAndAngle();
            frontLeft.setSpeedAndAngle();
            rearLeft.setSpeedAndAngle();
            rearRight.setSpeedAndAngle();
        }  
    }

    public void setCoastMode () {
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

    public void setBrakeMode () {
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
