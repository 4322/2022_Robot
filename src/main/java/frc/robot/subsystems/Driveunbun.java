package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.cameras.Webcams;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
import frc.robot.subsystems.SwerveDrive.TalonFXModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

public class Driveunbun extends SubsystemBase {

  private TalonFXModule[] swerveModules = new TalonFXModule[4];

  private AHRS gyro;

  private PIDController rotPID;
  private Webcams webcams;
  private Limelight limelight;

  private boolean tipDecelerateActive = false;
  private boolean tipSmallStickActive = false;
  private boolean tipBigStickActive = false;
  private Timer runTime = new Timer();
  private boolean ledsOff = false;

  private ArrayList<SnapshotVectorXY> velocityHistory = new ArrayList<SnapshotVectorXY>();

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

  public Driveunbun(Webcams webcams, Limelight limelight) {
    this.webcams = webcams;
    this.limelight = limelight;
    runTime.start();

    if (Constants.driveEnabled) {
      swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] = new TalonFXModule(
          DriveConstants.frontRightRotationID, DriveConstants.frontRightDriveID,
          WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
      swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] = new TalonFXModule(DriveConstants.frontLeftRotationID,
          DriveConstants.frontLeftDriveID,
          WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
      swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] = new TalonFXModule(DriveConstants.rearRightRotationID,
          DriveConstants.rearRightDriveID,
          WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID);
      swerveModules[WheelPosition.BACK_LEFT.wheelNumber] = new TalonFXModule(DriveConstants.rearLeftRotationID,
          DriveConstants.rearLeftDriveID,
          WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);
    }
  }

  public void init() {
    if (Constants.driveEnabled) {
      for (TalonFXModule module : swerveModules) {
        module.init();
      }

      rotPID = new PIDController(DriveConstants.autoRotkP, 0, DriveConstants.autoRotkD);
      SwerveHelper.setReversingToSpeed();

      if (Constants.gyroEnabled) {
        gyro = new AHRS(SPI.Port.kMXP);

        // wait for first gyro reading to be received
        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        resetFieldCentric();
        SwerveHelper.setGyro(gyro);
      } else {
        setDriveMode(DriveMode.frontCamCentric);
      }

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        errorDisplay = tab.add("Rot Error", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

        rotSpeedDisplay = tab.add("Rotation Speed", 0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

        rotkP = tab.add("Rotation kP", DriveConstants.autoRotkP)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

        rotkD = tab.add("Rotation kD", DriveConstants.autoRotkD)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();

        roll = tab.add("Roll", 0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

        pitch = tab.add("Pitch", 0)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();

        botVelocityMag = tab.add("Bot Vel Mag", 0)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();

        botAccelerationMag = tab.add("Bot Acc Mag", 0)
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();

        botVelocityAngle = tab.add("Bot Vel Angle", 0)
            .withPosition(4, 0)
            .withSize(1, 1)
            .getEntry();

        botAccelerationAngle = tab.add("Bot Acc Angle", 0)
            .withPosition(4, 1)
            .withSize(1, 1)
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
            .getEntry();
      }
    }
  }

  public enum DriveMode {
    fieldCentric(0),
    frontCamCentric(1),
    leftCamCentric(2),
    rightCamCentric(3),
    limelightFieldCentric(4),
    killFieldCentric(5),
    sideKillFieldCentric(6);

    private int value;

    DriveMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private static DriveMode driveMode;

  public static DriveMode getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
    switch (mode) {
      case fieldCentric:
      case limelightFieldCentric:
      case killFieldCentric:
        SwerveHelper.setToFieldCentric();
        webcams.resetCameras();
        break;
      case leftCamCentric:
        SwerveHelper.setToBotCentric(90);
        webcams.setLeft();
        break;
      case rightCamCentric:
        SwerveHelper.setToBotCentric(-90);
        webcams.setRight();
        break;
      case frontCamCentric:
        SwerveHelper.setToBotCentric(0);
        webcams.setFront();
        break;
      case sideKillFieldCentric:
      SwerveHelper.setToFieldCentric();
      webcams.setLeftAndRight();
      break;
    }
    if (mode == DriveMode.limelightFieldCentric) {
      limelight.setLed(LedMode.On);
    } else {
      limelight.setLed(LedMode.Off);
    }
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // acceleration must be calculated once and only once per periodic interval
    for (TalonFXModule module : swerveModules) {
      module.snapshotAcceleration();
    }

    // turn off limelight LEDs following power-up because the limelight takes longer
    // to boot than the roboRio
    if (!ledsOff && runTime.hasElapsed(30)) {
      setDriveMode(driveMode);
      ledsOff = true;
    }

    if (Constants.debug) { // don't combine if statements to avoid dead code warning
      if (Constants.gyroEnabled) {
        roll.setDouble(gyro.getRoll());
        pitch.setDouble(gyro.getPitch());
      }
    }
  }

  // make the current robot direction be forward
  public void resetFieldCentric() {
    if (gyro != null) {
      gyro.setAngleAdjustment(0);
      gyro.setAngleAdjustment(-gyro.getAngle());
    }
    setDriveMode(DriveMode.fieldCentric);
  }

  public void drive(double driveX, double driveY, double rotate) {
    if (Constants.driveEnabled) {
      double clock = runTime.get();  // cache value to reduce CPU usage
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
      velocityHistory.removeIf(n -> 
        (n.getTime() < clock - DriveConstants.Tip.velocityHistorySeconds));
      velocityHistory.add(new SnapshotVectorXY(velocityXY, clock));

      if (Constants.debug) {
        botVelocityMag.setDouble(velocity);
        botAccelerationMag.setDouble(acceleration);
        botVelocityAngle.setDouble(90 - velocityXY.degrees());
        botAccelerationAngle.setDouble(90 - accelerationXY.degrees());
      }

      // anti-tipping logic
      if (velocity >= (tipDecelerateActive ? DriveConstants.Tip.lowVelocityFtPerSec
                                           : DriveConstants.Tip.highVelocityFtPerSec) &&
          acceleration >= (tipDecelerateActive ? DriveConstants.Tip.lowAccFtPerSec2
                                               : DriveConstants.Tip.highAccFtPerSec2) &&
          // check if decelerating
          Math.abs(SwerveHelper.boundDegrees(180 +
              velocityXY.degrees() - accelerationXY.degrees())) <= DriveConstants.Tip.velAccDiffMaxDeg) {
        rotate = 0; // don't tip over our own wheels while declerating
        tipDecelerateActive = true;
      } else {
        tipDecelerateActive = false;
      }

      // Don't get stuck in anti-tipping mode if driver is applying partial power.
      // Scale power threshold based on the robot velocity to allow steering
      // at low power and low velocity.
      double powerOffThreshold = DriveConstants.Tip.lowPowerOff +
          (DriveConstants.Tip.highPowerOff - DriveConstants.Tip.lowPowerOff) *
              (velocity - DriveConstants.Tip.lowVelocityFtPerSec) /
              (DriveConstants.Tip.highVelocityFtPerSec - DriveConstants.Tip.lowVelocityFtPerSec);
      powerOffThreshold = Math.max(DriveConstants.Tip.lowPowerOff,
          Math.min(DriveConstants.Tip.highPowerOff, powerOffThreshold));

      if (velocity >= DriveConstants.Tip.lowVelocityFtPerSec &&
          driveXY.magnitude() < powerOffThreshold) {
        rotate = 0; // don't tip when about to start declerating
        tipSmallStickActive = true;
      } else {
        tipSmallStickActive = false;
      }

      // find largest recent steering change
      double maxSteeringChangeDegrees = 0;
      for (SnapshotVectorXY velocitySnapshot : velocityHistory) {
        double steeringChangeDegrees = driveXY.degrees() - velocitySnapshot.getVectorXY().degrees();
        if (SwerveHelper.isFieldCentric()) {
          steeringChangeDegrees += SwerveHelper.getGyroYawDeg();
        }
        steeringChangeDegrees = Math.abs(SwerveHelper.boundDegrees(steeringChangeDegrees));
        maxSteeringChangeDegrees = Math.max(maxSteeringChangeDegrees, steeringChangeDegrees);
      }

      // detect large changes in drive stick that would tip due to excessive wheel rotation
      if (!isDrivingWithSideCams() &&
          velocity >= DriveConstants.Tip.lowVelocityFtPerSec &&
          maxSteeringChangeDegrees >= DriveConstants.Tip.highSpeedSteeringChangeMaxDegrees &&
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
        for (TalonFXModule module : swerveModules) {
          module.setSpeedAndAngle();
        }
      }
    }
  }

  // Uses a PID Controller to rotate the robot to a certain degree
  // Must be periodically updated to work
  public void driveAutoRotate(double driveX, double driveY, double error) {

    if (Constants.debug) {
      rotPID.setP(rotkP.getDouble(DriveConstants.autoRotkP));
      rotPID.setD(rotkD.getDouble(DriveConstants.autoRotkD));
    }

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

  public boolean isDrivingWithSideCams() {
    return (driveMode == DriveMode.leftCamCentric) || (driveMode == DriveMode.rightCamCentric);
  }

  public void setCoastMode() {
    if (Constants.driveEnabled) {
      for (TalonFXModule module : swerveModules) {
        module.setCoastMode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (TalonFXModule module : swerveModules) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    for (TalonFXModule module : swerveModules) {
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

  private class SnapshotVectorXY {
    private VectorXY vectorXY;
    private double time;

    public SnapshotVectorXY(VectorXY vectorXY, double time) {
      this.vectorXY = vectorXY;
      this.time = time;
    }

    public VectorXY getVectorXY() {
      return vectorXY;
    }

    public double getTime() {
      return time;
    }
  }
}
