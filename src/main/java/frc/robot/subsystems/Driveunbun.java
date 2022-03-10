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
    private NetworkTableEntry rotkP;
    private NetworkTableEntry rotkD;
    private NetworkTableEntry roll;
    private NetworkTableEntry pitch;

    public Driveunbun() {
        if (Constants.driveEnabled) {
            frontRightDrive = new WPI_TalonFX(DriveConstants.frontRightDriveID);
            frontLeftDrive = new WPI_TalonFX(DriveConstants.frontLeftDriveID);
            rearRightDrive = new WPI_TalonFX(DriveConstants.rearRightDriveID);
            rearLeftDrive = new WPI_TalonFX(DriveConstants.rearLeftDriveID);
            frontRightRotation = new WPI_TalonFX(DriveConstants.frontRightRotationID);
            frontLeftRotation = new WPI_TalonFX(DriveConstants.frontLeftRotationID);
            rearRightRotation = new WPI_TalonFX(DriveConstants.rearRightRotationID);
            rearLeftRotation = new WPI_TalonFX(DriveConstants.rearLeftRotationID);

            frontRight = new TalonFXModule(frontRightRotation, frontRightDrive, 
                WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
            frontLeft = new TalonFXModule(frontLeftRotation, frontLeftDrive, 
                WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
            rearRight = new TalonFXModule(rearRightRotation, rearRightDrive, 
                WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID); 
            rearLeft = new TalonFXModule(rearLeftRotation, rearLeftDrive, 
                WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);   
         }
    }

    public void init() {
        if (Constants.driveEnabled) {
            frontRight.init();
            frontLeft.init();
            rearRight.init();
            rearLeft.init();

            rotPID = new PIDController(DriveConstants.autoRotkP, 0, DriveConstants.autoRotkD);

            if (Constants.gyroEnabled) {
                gyro = new AHRS(SPI.Port.kMXP);

                // wait for first gyro reading to be received
                try {
                    Thread.sleep(250);
                }
                catch (InterruptedException e) {}

                resetFieldCentric();
                SwerveHelper.setGyro(gyro);
            }

            if (Constants.debug) {
                tab = Shuffleboard.getTab("Drivebase");

                errorDisplay = tab.add("Rot Error", 0)
                .withPosition(0,0)   
                .withSize(1,1)
                .getEntry();

                rotSpeedDisplay = tab.add("Rotation Speed", 0)
                .withPosition(0,1)   
                .withSize(1,1)
                .getEntry();

                rotkP = tab.add("Rotation kP", DriveConstants.autoRotkP)
                .withPosition(1,0)   
                .withSize(1,1)
                .getEntry();

                rotkD = tab.add("Rotation kD", DriveConstants.autoRotkD)
                .withPosition(2,0)   
                .withSize(1,1)
                .getEntry();

                roll = tab.add("Roll", 0)
                .withPosition(1,1)
                .withSize(1,1)
                .getEntry();

                pitch = tab.add("Pitch", 0)
                .withPosition(2,1)
                .withSize(1,1)
                .getEntry();
            }

            SwerveHelper.setReversingToSpeed();
        }   
    }

    @Override
    public void periodic() {
        if (Constants.debug) {  // don't combine if statements to avoid dead code warning
            if (Constants.gyroEnabled) {
                roll.setDouble(gyro.getRoll());
                pitch.setDouble(gyro.getPitch());
            }
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
    
            if (rotate > DriveConstants.maxAutoRotSpd) {
                rotate = DriveConstants.maxAutoRotSpd;
            } else if (rotate < -DriveConstants.maxAutoRotSpd) {
                rotate = -DriveConstants.maxAutoRotSpd;
            }

            SwerveHelper.calculate(
                    driveX, driveY, rotate, currentAngle
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

        if (Constants.debug) {
            rotPID.setP(rotkP.getDouble(DriveConstants.autoRotkP));
            rotPID.setD(rotkD.getDouble(DriveConstants.autoRotkD));
        }

        double error = SwerveHelper.boundDegrees(autoRotateDeg - gyro.getAngle());
        double rotPIDSpeed = rotPID.calculate(error, 0);

        if (error <= DriveConstants.autoRotateToleranceDegrees) {
            rotPIDSpeed = 0;
        }

        drive(driveX, driveY, rotPIDSpeed);

        if (Constants.debug) {
            errorDisplay.setDouble(error);
            rotSpeedDisplay.setDouble(rotPIDSpeed);
        }
    }

    // Drives the robot at a certain angle (relative to the field, forward = 0 deg)
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

    public void stop() {
        frontRight.stop();
        frontLeft.stop();
        rearRight.stop();
        rearLeft.stop();
    }
}
