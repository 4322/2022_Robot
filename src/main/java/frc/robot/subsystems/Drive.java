package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory.State;
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
import frc.robot.subsystems.SwerveDrive.SwerveModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private AHRS gyro;

  private PIDController rotPID;
  private Webcams webcams;
  private Limelight limelight;

  private boolean tipDecelerateActive = false;
  private boolean tipSmallStickActive = false;
  private boolean tipBigStickActive = false;
  private Timer runTime = new Timer();
  private boolean ledsOff = false;
  private double latestVelocity;
  private double latestAcceleration;

  private ArrayList<SnapshotVectorXY> velocityHistory = new ArrayList<SnapshotVectorXY>();

  private final Translation2d frontLeftLocation = new Translation2d(Constants.DriveConstants.distWheelMetersX, Constants.DriveConstants.distWheelMetersY);
  private final Translation2d frontRightLocation = new Translation2d(Constants.DriveConstants.distWheelMetersX, -Constants.DriveConstants.distWheelMetersY);
  private final Translation2d backLeftLocation = new Translation2d(-Constants.DriveConstants.distWheelMetersX, Constants.DriveConstants.distWheelMetersY);
  private final Translation2d backRightLocation = new Translation2d(-Constants.DriveConstants.distWheelMetersX, -Constants.DriveConstants.distWheelMetersY);

  private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            // wheel locations must be in the same order as the WheelPosition enum values
            frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);

  private RamseteController ram = new RamseteController();
            
  private SwerveDriveOdometry odometry;
  private double robotCentricOffsetDegrees;
  private boolean fieldRelative;
  
  private ShuffleboardTab tab;
  private NetworkTableEntry rotErrorTab;
  private NetworkTableEntry rotSpeedTab;
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
  private NetworkTableEntry driveXTab;
  private NetworkTableEntry driveYTab;
  private NetworkTableEntry rotateTab;
  private NetworkTableEntry displaceXTab;
  private NetworkTableEntry displaceYTab;

  public Drive(Webcams webcams, Limelight limelight) {
    this.webcams = webcams;
    this.limelight = limelight;
    runTime.start();
  }

  public void init() {
    if (Constants.driveEnabled) {

      rotPID = new PIDController(DriveConstants.autoRotkP, 0, DriveConstants.autoRotkD);

      if (Constants.gyroEnabled) {
        gyro = new AHRS(SPI.Port.kMXP);

        // wait for first gyro reading to be received
        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        resetFieldCentric(0);
      } else {
        setDriveMode(DriveMode.frontCamCentric);
      }

      if (Constants.driveEnabled) {
        swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] = new SwerveModule(
            DriveConstants.frontRightRotationID, DriveConstants.frontRightDriveID,
            WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
        swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] = new SwerveModule(DriveConstants.frontLeftRotationID,
            DriveConstants.frontLeftDriveID,
            WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
        swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] = new SwerveModule(DriveConstants.rearRightRotationID,
            DriveConstants.rearRightDriveID,
            WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID);
        swerveModules[WheelPosition.BACK_LEFT.wheelNumber] = new SwerveModule(DriveConstants.rearLeftRotationID,
            DriveConstants.rearLeftDriveID,
            WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

        for (SwerveModule module : swerveModules) {
          module.init();
        }
      }

      ram.setTolerance(DriveConstants.poseError);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        rotErrorTab = tab.add("Rot Error", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

        rotSpeedTab = tab.add("Rotation Speed", 0)
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

        driveXTab = tab.add("Drive X", 0)
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();

        driveYTab = tab.add("Drive Y", 0)
        .withPosition(1, 2)
        .withSize(1, 1)
        .getEntry();

        rotateTab = tab.add("Rotate", 0)
        .withPosition(1, 3)
        .withSize(1, 1)
        .getEntry();

        displaceXTab = tab.add("Displacement X", 0)
        .withPosition(3, 2)
        .withSize(1, 1)
        .getEntry();

        displaceYTab = tab.add("Displacement Y", 0)
        .withPosition(4, 2)
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
    if (Constants.demo.inDemoMode) {
      if (mode != DriveMode.fieldCentric) {  // split if statements to avoid deadcode warning in normal mode
        return;
      }
    }
    driveMode = mode;
    switch (mode) {
      case fieldCentric:
      case limelightFieldCentric:
      case killFieldCentric:
        setToFieldCentric();
        webcams.resetCameras();
        break;
      case leftCamCentric:
        setToBotCentric(90);
        webcams.setLeft();
        break;
      case rightCamCentric:
        setToBotCentric(-90);
        webcams.setRight();
        break;
      case frontCamCentric:
        setToBotCentric(0);
        webcams.setFront();
        break;
      case sideKillFieldCentric:
      setToFieldCentric();
      webcams.setLeftAndRight();
      break;
    }
    if (mode == DriveMode.limelightFieldCentric) {
      limelight.enableLed();
    } else {
      limelight.disableLed();
    }
  }

  // Positive gyro angle is CW
  public double getAngle(){
		if (gyro != null && gyro.isConnected() && !gyro.isCalibrating()) {
			return -gyro.getAngle();
		}
		else {
			return 0;
		}
	}

  @Override
  public void periodic() {
    if (Constants.driveEnabled) {
      // acceleration must be calculated once and only once per periodic interval
      for (SwerveModule module : swerveModules) {
        module.snapshotAcceleration();
      }

      updateOdometry();
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
        displaceXTab.setDouble(gyro.getDisplacementX());
        displaceYTab.setDouble(gyro.getDisplacementY());
      }
    }
  }

  public void resetDisplacement() {
    if (Constants.gyroEnabled) {
      gyro.resetDisplacement();
    }
  }

  // rotation isn't considered to be movement
  public boolean isRobotMoving() {
    return latestVelocity >= DriveConstants.movingVelocityThresholdFtPerSec;
  }

  // make the current robot direction be forward, or add an offset
  public void resetFieldCentric(double offset) {
    if (gyro != null) {
      gyro.setAngleAdjustment(0);
      gyro.setAngleAdjustment(-gyro.getAngle() + offset);
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
        double wheelAngleDegrees = currentAngle[i];
        velocityXY.add(new VectorPolarDegrees(
            swerveModules[i].getVelocity(),
            wheelAngleDegrees));
        accelerationXY.add(new VectorPolarDegrees(
            swerveModules[i].getAcceleration(),
            wheelAngleDegrees));
      }
      latestVelocity = velocityXY.magnitude() / 4;
      latestAcceleration = accelerationXY.magnitude() / 4;
      velocityHistory.removeIf(n -> 
        (n.getTime() < clock - DriveConstants.Tip.velocityHistorySeconds));
      velocityHistory.add(new SnapshotVectorXY(velocityXY, clock));

      if (Constants.debug) {
        botVelocityMag.setDouble(latestVelocity);
        botAccelerationMag.setDouble(latestAcceleration);
        botVelocityAngle.setDouble(velocityXY.degrees());
        botAccelerationAngle.setDouble(accelerationXY.degrees());
        driveXTab.setDouble(driveX);
        driveYTab.setDouble(driveY);
        rotateTab.setDouble(rotate);
      }

      // anti-tipping logic
      if (latestVelocity >= (tipDecelerateActive ? DriveConstants.Tip.lowVelocityFtPerSec
                                           : DriveConstants.Tip.highVelocityFtPerSec) &&
          latestAcceleration >= (tipDecelerateActive ? DriveConstants.Tip.lowAccFtPerSec2
                                               : DriveConstants.Tip.highAccFtPerSec2) &&
          // check if decelerating
          Math.abs(boundDegrees(180 +
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
              (latestVelocity - DriveConstants.Tip.lowVelocityFtPerSec) /
              (DriveConstants.Tip.highVelocityFtPerSec - DriveConstants.Tip.lowVelocityFtPerSec);
      powerOffThreshold = Math.max(DriveConstants.Tip.lowPowerOff,
          Math.min(DriveConstants.Tip.highPowerOff, powerOffThreshold));

      if (latestVelocity >= DriveConstants.Tip.lowVelocityFtPerSec &&
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
        if (fieldRelative) {
          steeringChangeDegrees -= getAngle();
        }
        steeringChangeDegrees = Math.abs(boundDegrees(steeringChangeDegrees));
        maxSteeringChangeDegrees = Math.max(maxSteeringChangeDegrees, steeringChangeDegrees);
      }

      // detect large changes in drive stick that would tip due to excessive wheel rotation
      if (!isDrivingWithSideCams() &&
          latestVelocity >= DriveConstants.Tip.lowVelocityFtPerSec &&
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

      // don't allow the wheels to rotate back to 45 deg angle
      if (tipDecelerateActive || tipSmallStickActive || tipBigStickActive) {
        rotate = 0;
      }

      // convert to proper units
      rotate = rotate * DriveConstants.maxRotationSpeedRadSecond;
      driveX = driveX * DriveConstants.maxSpeedMetersPerSecond;
      driveY = driveY * DriveConstants.maxSpeedMetersPerSecond;

      // ready to drive!
      if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
        // don't rotate wheels such that we trip over them when decelerating
        stop();
      } else {
        Rotation2d robotAngle;
        if (fieldRelative && Constants.gyroEnabled) {
          robotAngle = gyro.getRotation2d();
        } else {
          robotAngle = Rotation2d.fromDegrees(-robotCentricOffsetDegrees);
        }
        // create SwerveModuleStates inversely from the kinematics
        var swerveModuleStates =
            kinematics.toSwerveModuleStates(
              ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, robotAngle));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.maxSpeedMetersPerSecond);
        for (int i = 0; i < swerveModules.length; i++) {
          swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
      }
    }
  }

  // Drive using State object
  public void drive(State trajectoryState) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(ram.calculate(odometry.getPoseMeters(), trajectoryState));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.maxSpeedMetersPerSecond);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public boolean isAtTarget() {
    return ram.atReference();
  }

  // Uses a PID Controller to rotate the robot to a certain degree
  // Must be periodically updated to work
  public void driveAutoRotate(double driveX, double driveY, double rotateDeg, double toleranceDeg) {

    if (Constants.debug) {
      rotPID.setP(rotkP.getDouble(DriveConstants.autoRotkP));
      rotPID.setD(rotkD.getDouble(DriveConstants.autoRotkD));
    }

    double rotPIDSpeed = rotPID.calculate(0, rotateDeg);

    if (Math.abs(rotateDeg) <= toleranceDeg) {
      rotPIDSpeed = 0;
    } else if (Math.abs(rotPIDSpeed) < DriveConstants.minAutoRotateSpeed) {
      rotPIDSpeed = Math.copySign(DriveConstants.minAutoRotateSpeed, rotPIDSpeed);
    } else if (rotPIDSpeed > DriveConstants.maxAutoRotateSpeed) {
      rotPIDSpeed = DriveConstants.maxAutoRotateSpeed;
    } else if (rotPIDSpeed < -DriveConstants.maxAutoRotateSpeed) {
      rotPIDSpeed = -DriveConstants.maxAutoRotateSpeed;
    }

    drive(driveX, driveY, rotPIDSpeed);

    if (Constants.debug) {
      rotErrorTab.setDouble(rotateDeg);
      rotSpeedTab.setDouble(rotPIDSpeed);
    }
  }

  // To be called at the start of a set rotation movement.
  // Do not use for continously changing rotations.
  public void resetRotatePID() {
    rotPID.reset();
  }

  public void updateOdometry() {
    if (Constants.gyroEnabled) {
        odometry.update(
            gyro.getRotation2d(),
            // wheel locations must be in the same order as the WheelPosition enum values
            swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber].getState(),
            swerveModules[WheelPosition.FRONT_LEFT.wheelNumber].getState(),
            swerveModules[WheelPosition.BACK_LEFT.wheelNumber].getState(),
            swerveModules[WheelPosition.BACK_RIGHT.wheelNumber].getState()
            );
        }
    }

  	public void setToFieldCentric(){
      fieldRelative = true;
      robotCentricOffsetDegrees = 0;
    }
  
    public void setToBotCentric(double offsetDeg){
      fieldRelative = false;
      robotCentricOffsetDegrees = offsetDeg;
    }

  public boolean isDrivingWithSideCams() {
    return (driveMode == DriveMode.leftCamCentric) || (driveMode == DriveMode.rightCamCentric);
  }

  public void setCoastMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setCoastMode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    for (SwerveModule module : swerveModules) {
      module.stop();
    }
  }

  //TODO: do this in a way that doesnt break encapsulation
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
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

	// convert angle to range of +/- 180 degrees
	public static double boundDegrees(double angleDegrees) {
		double x = ((angleDegrees + 180) % 360) - 180;
		if (x < -180) {
			x += 360;
		}
		return x;
	}
}
