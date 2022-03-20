package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Drive;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
import frc.robot.subsystems.SwerveDrive.TalonFXModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

public class Driveunbun extends SubsystemBase {

    private TalonFXModule[] swerveModules = new TalonFXModule[4];

    private AHRS gyro;

    private PIDController rotPID;

    private boolean drivingWithSideCams = false;
    private boolean tipDecelerateActive = false;
    private boolean tipSmallStickActive = false;
    private boolean tipBigStickActive = false;

    private ShuffleboardTab tab;
    private NetworkTableEntry errorDisplay;
    private NetworkTableEntry rotSpeedDisplay;
    private NetworkTableEntry rotkP;
    private NetworkTableEntry rotkD;
    private NetworkTableEntry roll;
    private NetworkTableEntry pitch;
    private NetworkTableEntry botVelocityMag;
    private NetworkTableEntry botAccelerationMag;
    private NetworkTableEntry botVelocityAngle;
    private NetworkTableEntry botAccelerationAngle;
    private NetworkTableEntry tipDecelerationAtiveTab;
    private NetworkTableEntry tipSmallStickAtiveTab;
    private NetworkTableEntry tipBigStickAtiveTab;

    public Driveunbun() {
        if (Constants.driveEnabled) {
            swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] = 
                new TalonFXModule(DriveConstants.frontRightRotationID, DriveConstants.frontRightDriveID, 
                WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
            swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] = 
                new TalonFXModule(DriveConstants.frontLeftRotationID, DriveConstants.frontLeftDriveID, 
                WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
            swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] = 
                new TalonFXModule(DriveConstants.rearRightRotationID, DriveConstants.rearRightDriveID, 
                WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID);
            swerveModules[WheelPosition.BACK_LEFT.wheelNumber] = 
                new TalonFXModule(DriveConstants.rearLeftRotationID, DriveConstants.rearLeftDriveID, 
                WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);
         }
    }

    public void init() {
        if (Constants.driveEnabled) {
            for (TalonFXModule module:swerveModules) {
                module.init();
            }

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
            SwerveHelper.setReversingToSpeed();

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

                botVelocityMag = tab.add("Bot Vel Mag", 0)
                .withPosition(3,0)
                .withSize(1,1)
                .getEntry();

                botAccelerationMag = tab.add("Bot Acc Mag", 0)
                .withPosition(3,1)
                .withSize(1,1)
                .getEntry();

                botVelocityAngle = tab.add("Bot Vel Angle", 0)
                .withPosition(4,0)
                .withSize(1,1)
                .getEntry();

                botAccelerationAngle = tab.add("Bot Acc Angle", 0)
                .withPosition(4,1)
                .withSize(1,1)
                .getEntry();        
                
                tipDecelerationAtiveTab = tab.add("Tip Deceleration", true)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(5, 0)
                .withSize(1, 1)
                .getEntry();

                tipSmallStickAtiveTab = tab.add("Tip Small Stick", true)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

                tipBigStickAtiveTab = tab.add("Tip Big Stick", true)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(5, 2)
                .withSize(1, 1)
                .getEntry();            }
        }   
    }

    @Override
    public void periodic() {
        // acceleration must be calculated once and only once per periodic interval
        for (TalonFXModule module:swerveModules) {
            module.snapshotAcceleration();
        }

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
            drivingWithSideCams = false;
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

    public void setToRobotCentric(double offsetDeg) {
        // positive offset = counter-clockwise
        if (offsetDeg != 0) {
            drivingWithSideCams = true;
        } else {
            drivingWithSideCams = false;
        }
        SwerveHelper.setToBotCentric(offsetDeg);
    }

    public void drive(double driveX, double driveY, double rotate) {
        if (Constants.driveEnabled) {
            double[] currentAngle = new double[4];
            for (int i = 0; i < swerveModules.length; i++) {
                currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
            }

            VectorXY velocityXY = new VectorXY();
            VectorXY accelerationXY = new VectorXY();
            VectorXY driveXY = new VectorXY(driveX, driveY);

            // sum wheel velocity and acceleration vectors
            for (int i = 0; i < swerveModules.length; i++) {
                double wheelAngleDegrees = 90 - currentAngle[i];
                velocityXY.add(new VectorPolarDegrees(
                    swerveModules[i].getVelocity(), 
                    wheelAngleDegrees));
                accelerationXY.add(new VectorPolarDegrees(
                    swerveModules[i].getAcceleration(), 
                    wheelAngleDegrees));            
            }            
            double velocity = velocityXY.magnitude() / 4;
            double acceleration = accelerationXY.magnitude() / 4;

            if (Constants.debug) {
                botVelocityMag.setDouble(velocity);
                botAccelerationMag.setDouble(acceleration);
                botVelocityAngle.setDouble(90 - velocityXY.degrees());
                botAccelerationAngle.setDouble(90 - accelerationXY.degrees());
            }

            // anti-tipping logic
            if (velocity >= (tipDecelerateActive? DriveConstants.Tip.lowVelocityFtperSec : 
                                                  DriveConstants.Tip.highVelocityFtperSec) &&
                acceleration >= (tipDecelerateActive? DriveConstants.Tip.lowAccFtPerSec2 :
                                                      DriveConstants.Tip.highAccFtPerSec2) &&
                // check if decelerating
                Math.abs(SwerveHelper.boundDegrees(180 +
                    velocityXY.degrees() - accelerationXY.degrees())) <=
                    DriveConstants.Tip.velAccDiffMaxDeg) {
                rotate = 0;  // don't tip over our own wheels while declerating
                tipDecelerateActive = true;
            } else {
                tipDecelerateActive = false;
            }

            // Don't get stuck in anti-tipping mode if driver is applying partial power.
            // Scale power threshold based on the robot velocity to allow steering
            // at low power and low velocity.
            double powerOffThreshold = DriveConstants.Tip.lowPowerOff + 
                (DriveConstants.Tip.highPowerOff - DriveConstants.Tip.lowPowerOff) *
                (velocity - DriveConstants.Tip.lowVelocityFtperSec) /
                (DriveConstants.Tip.highVelocityFtperSec - DriveConstants.Tip.lowVelocityFtperSec);
            powerOffThreshold = Math.max(DriveConstants.Tip.lowPowerOff, 
                                Math.min(DriveConstants.Tip.highPowerOff, powerOffThreshold));

            if (velocity >= DriveConstants.Tip.lowVelocityFtperSec &&
                    driveXY.magnitude() < powerOffThreshold) {
                rotate = 0;  // don't tip when about to start declerating
                tipSmallStickActive = true;
            } else {
                tipSmallStickActive = false;
            }

            // detect large changes in drive stick that would tip due to excessive wheel rotation
            double steeringChangeDegrees = Math.abs(SwerveHelper.boundDegrees(
                velocityXY.degrees() - driveXY.degrees()));

            if (velocity >= DriveConstants.Tip.lowVelocityFtperSec &&
                    steeringChangeDegrees > DriveConstants.Tip.highSpeedSteeringChangeMaxDegrees &&
                    driveXY.magnitude() >= DriveConstants.Tip.highPowerOff) {
                // kill power until velocity is low enough to allow turning to the new
                // direction without risking a tipover
                driveX = 0;
                driveY = 0;
                rotate = 0;
                tipBigStickActive = true;
            } else {
                tipBigStickActive = false;
            }            

            if (Constants.debug) {
                tipDecelerationAtiveTab.setBoolean(!tipDecelerateActive);
                tipSmallStickAtiveTab.setBoolean(!tipSmallStickActive);
                tipBigStickAtiveTab.setBoolean(!tipBigStickActive);
            }

            // ready to drive!
            if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
                // don't rotate wheels such that we trip over them when decelerating
                stop();
            } else {
                SwerveHelper.calculate(driveX, driveY, rotate, currentAngle);
                if (tipDecelerateActive || tipSmallStickActive || tipBigStickActive) {
                    SwerveHelper.noSteering();
                }
                for (TalonFXModule module:swerveModules) {
                    module.setSpeedAndAngle();
                }
            }
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

        if (Math.abs(error) <= DriveConstants.autoRotateToleranceDegrees) {
            rotPIDSpeed = 0;
        }
            
        if (rotPIDSpeed > DriveConstants.autoRotationMaxSpeed) {
            rotPIDSpeed = DriveConstants.autoRotationMaxSpeed;
        } else if (rotPIDSpeed < -DriveConstants.autoRotationMaxSpeed) {
            rotPIDSpeed = -DriveConstants.autoRotationMaxSpeed;
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

    public boolean getDrivingWithSideCams() {
        return drivingWithSideCams;
    }

    public void setCoastMode() {
        if (Constants.driveEnabled) {
            for (TalonFXModule module:swerveModules) {
                module.setCoastMode();
            }
        }
    }

    public void setBrakeMode () {
        if (Constants.driveEnabled) {
            for (TalonFXModule module:swerveModules) {
                module.setBrakeMode();
            }
        }
    } 

    public void stop() {
        for (TalonFXModule module:swerveModules) {
            module.stop();
        }
    }

    public class VectorXY extends Vector2d {

        public VectorXY() {
            super();
        }

        public VectorXY(double x, double y) {
            super(x, y);
        }

        public void add(Vector2d vec) {
            x += vec.x;
            y += vec.y; 
        }

        public double degrees() {
            return Math.toDegrees(Math.atan2(y, x));
        }
    }

    public class VectorPolarDegrees extends VectorXY {

        public VectorPolarDegrees(double r, double theta) {
            x = r * Math.cos(Math.toRadians(theta));
            y = r * Math.sin(Math.toRadians(theta));
        }
    }
}
