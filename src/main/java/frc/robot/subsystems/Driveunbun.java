package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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

    private AHRS gyro;

    private PIDController rotPID;

    private ShuffleboardTab tab;
    private NetworkTableEntry errorDisplay;
    private NetworkTableEntry rotSpeedDisplay;

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

            rotPID = new PIDController(DriveConstants.autoRotkP, 0, DriveConstants.autoRotkD);
            
            frontRight = new TalonFXModule(frontRightRotation, frontRightDrive, 
                WheelPosition.FRONT_RIGHT, Constants.DriveConstants.frontRightEncoderID);
            frontLeft = new TalonFXModule(frontLeftRotation, frontLeftDrive, 
                WheelPosition.FRONT_LEFT, Constants.DriveConstants.frontLeftEncoderID);
            rearRight = new TalonFXModule(rearRightRotation, rearRightDrive, 
                WheelPosition.BACK_RIGHT, Constants.DriveConstants.rearRightEncoderID); 
            rearLeft = new TalonFXModule(rearLeftRotation, rearLeftDrive, 
                WheelPosition.BACK_LEFT, Constants.DriveConstants.rearLeftEncoderID);

            if (Constants.gyroEnabled) {
                gyro = new AHRS(SPI.Port.kMXP);

                // wait for first gyro reading to be received
                try {
                    Thread.sleep(250);
                }
                catch (InterruptedException e) {}

                resetFieldCentric();
                SwerveHelper.setGyro(gyro);
                if (Constants.driveEnabled) {
                    tab = Shuffleboard.getTab("Drivebase");
                    errorDisplay = tab.add("Rot Error", 0)
                    .withPosition(0,0)   
                    .withSize(1,1)
                    .getEntry();
                    rotSpeedDisplay = tab.add("Rot Speed Display", 0)
                    .withPosition(0,1)   
                    .withSize(1,1)
                    .getEntry();
                }
            }

            SwerveHelper.setReversingToSpeed();
        }   
    }

    // activate field centric driving using the previously set forward orientation
    public void setToFieldCentric() {
        if (gyro != null) {
            SwerveHelper.setToFieldCentric();
        }
    }

    // make the current robot direction be forward
    public void resetFieldCentric() {
        if (gyro != null) {
            gyro.setAngleAdjustment(0);
            gyro.setAngleAdjustment(-gyro.getAngle()); 
            setToFieldCentric(); 
        }
    }

    public void setToRobotCentric() {
        SwerveHelper.setToBotCentric();
    }

    public void drive(double driveX, double driveY, double rotate) {
        if (Constants.driveEnabled) {
            double[] currentAngle = new double[4];
            currentAngle[WheelPosition.FRONT_RIGHT.wheelNumber] = frontRight.getInternalRotationDegrees();
            currentAngle[WheelPosition.FRONT_LEFT.wheelNumber] = frontLeft.getInternalRotationDegrees();
            currentAngle[WheelPosition.BACK_RIGHT.wheelNumber] = rearRight.getInternalRotationDegrees();
            currentAngle[WheelPosition.BACK_LEFT.wheelNumber] = rearLeft.getInternalRotationDegrees();
    
            SwerveHelper.calculate(
                    -driveX, driveY, -rotate, currentAngle
            );
            
            frontRight.setSpeedAndAngle();
            frontLeft.setSpeedAndAngle();
            rearLeft.setSpeedAndAngle();
            rearRight.setSpeedAndAngle();
        }  
    }

    // Uses a PID Controller to rotate the robot to a certain degree
    // Must be periodically updated to work
    public void driveAutoRotate(double driveX, double driveY, double autoRotateDeg) {
        double error = SwerveHelper.boundDegrees(autoRotateDeg - gyro.getAngle());
        double rotPIDSpeed = rotPID.calculate(error, 0);
        drive(driveX, driveY, rotPIDSpeed);
        errorDisplay.setDouble(error);
        rotSpeedDisplay.setDouble(rotPIDSpeed);
    }

    // Drives the robot at a certain angle (relative to front of robot)
    // Must be periodically updated to work
    public void drivePolar(double angle, double speed, double rotationDeg) {
        angle = Math.toRadians(-angle + 90); // adjust for front of robot as 0 and clockwise rotation
        double x = speed * Math.cos(angle);
        double y = speed * Math.sin(angle);
        driveAutoRotate(x, y, rotationDeg);
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
