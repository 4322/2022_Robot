// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.FiringSolution.FiringSolution;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean debug = false;

  public static final class demo {
    public enum DriveMode {
      OFF,
      SLOW_ROTATE_ONLY,
      SLOW_DRIVE
    }
    public enum ShootingSpeed {
      EJECT_ONLY,
      SLOW_POP,
      FENDER,
      MEDIUM
    }
    public static final boolean inDemoMode = false;
    public static final DriveMode driveMode = DriveMode.SLOW_DRIVE;
    public static final ShootingSpeed shootingSpeed = ShootingSpeed.MEDIUM;

    public static final double driveScaleFactor = 0.15;
    public static final double rotationScaleFactor = 0.1;
  }

  public static final boolean driveEnabled = (demo.driveMode != demo.DriveMode.OFF)  || !demo.inDemoMode;
  public static final boolean joysticksEnabled = true;
  public static final boolean gyroEnabled = true;
  public static final boolean intakeEnabled = true;
  public static final boolean shooterEnabled = true;
  public static final boolean kickerEnabled = true;
  public static final boolean climberEnabled = true;
  public static final boolean hoodEnabled = true;
  public static final boolean conveyorEnabled = true;
  public static final boolean ballSensorEnabled = true;
  public static final boolean limelightEnabled = true;

  // configuration values common to all motor controllers
  public static final int controllerConfigTimeoutMs = 50;
  public static final int fastStatusPeriodBaseMs = 13;
  public static final int fastStatusPeriodMaxMs = 18;  // less than 20ms periodic code cycle time
  public static final int shuffleboardStatusPeriodBaseMs = 75;
  public static final int shuffleboardStatusPeriodMaxMs = 90;  // for interactive response
  public static final int slowStatusPeriodBaseMs = 180;  // for non-essential status
  public static final int slowStatusPeriodMaxMs = 255;  // avoid Talon 8-bit wrapping of status period
  // SPARK controllers support status periods of up to 65535 ms
  public static final int verySlowStatusPeriodSparkBaseMs = 1000;  // for unused status

  public static final int falconEncoderResolution = 2048;

  // Controller commands are transmited by the library on the CAN bus every 10 ms.
  // We then need to wait for the next status frame 
  // for the values we read from the library to be current.
  // We wait for two status frame periods in case the first frame was dropped.
  public static final double statusLatencySec = (10 + Constants.fastStatusPeriodMaxMs * 2) / 1000;

  public static final double hoodFiringSolutionOffset = 500;

  // Firing Solutions
  // fender distances need to be remeasured
  // measurement from back of bumper for now
  public static final class FiringSolutions {
    public static final FiringSolution fenderHigh = new FiringSolution(2200, 3000, 1800 + hoodFiringSolutionOffset, 3.0, 0);
    public static final FiringSolution fenderLow = new FiringSolution(1400, 1400, 8700 + hoodFiringSolutionOffset, 5.0, 0);
    public static final FiringSolution disposal = new FiringSolution(600, 700, 2500 + hoodFiringSolutionOffset, 5.0, 0);
    public static final FiringSolution middleTarmac = new FiringSolution(2200, 3000, 2500 + hoodFiringSolutionOffset, 2.0, 44.6);
    public static final FiringSolution insideTarmac = new FiringSolution(2200, 3100, 2800 + hoodFiringSolutionOffset, 2.0, 53);
    public static final FiringSolution outsideTarmac = new FiringSolution(2400, 3300, 4000 + hoodFiringSolutionOffset, 1.5, 96.8);
    public static final FiringSolution cargoRing = new FiringSolution(2600, 3400, 4000 + hoodFiringSolutionOffset, 1.5, 128);
    public static final FiringSolution closeLaunchpad = new FiringSolution(2600, 3650, 4000 + hoodFiringSolutionOffset, 1.5, 169.5);
    public static final FiringSolution autoBall5 = new FiringSolution(3200, 4100, 4200 + hoodFiringSolutionOffset, 1.25, 160);  // needs calibration
    public static final FiringSolution farLaunchpad = new FiringSolution(2600, 4000, 4100 + hoodFiringSolutionOffset, 1.0, 233.75);
    public static final FiringSolution reallyFar = new FiringSolution(2600, 4200, 4300 + hoodFiringSolutionOffset, 1.0, 271.75);
    public static final FiringSolution demoSlowPop = new FiringSolution(1500, 1700, 1400 + hoodFiringSolutionOffset, 1.0, 0);
    public static final FiringSolution demoMediumPop = new FiringSolution(2200, 3000, 1800 + hoodFiringSolutionOffset, 1.0, 0);
  }

  public static FiringSolution[] limelightFiringSolutions;
  static {  // limelight can't see target from the fender
      limelightFiringSolutions = new FiringSolution[7];
      int i = 0;
      limelightFiringSolutions[i++] = FiringSolutions.middleTarmac;
      limelightFiringSolutions[i++] = FiringSolutions.insideTarmac;
      limelightFiringSolutions[i++] = FiringSolutions.outsideTarmac;
      limelightFiringSolutions[i++] = FiringSolutions.cargoRing;
      limelightFiringSolutions[i++] = FiringSolutions.closeLaunchpad;
      limelightFiringSolutions[i++] = FiringSolutions.farLaunchpad;
      limelightFiringSolutions[i++] = FiringSolutions.reallyFar;
  }

  public static final class DriveConstants {
      public static final int frontRightDriveID = 2;
      public static final int frontRightRotationID = 3;
      public static final int rearRightDriveID = 4;
      public static final int rearRightRotationID = 5;
      public static final int frontLeftDriveID = 6;
      public static final int frontLeftRotationID = 7;
      public static final int rearLeftDriveID = 8;
      public static final int rearLeftRotationID = 9;

      public static final int frontRightEncoderID = 10;
      public static final int rearRightEncoderID = 11;
      public static final int frontLeftEncoderID = 12;
      public static final int rearLeftEncoderID = 13;

      public static final double kMaxSpeed = 3.0;
      public static final double kMaxAngularSpeed = Math.PI;
      public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

      public static final double distWheelX = 0.339725;
      public static final double distWheelY = 0.244475;

      public static final double wheelBaseLengthFeet = 26.75 / 12.0;
      public static final double wheelBaseWidthFeet = 19.25 / 12.0;

      public static final double autoRotkP = 0.008;
      public static final double autoRotkD = 0.0004;
      public static final double manualRotateToleranceDegrees = 1.5;
      public static final double autoRotateToleranceDegrees = 1.0;
      public static final double limeRotNotMovingToleranceDegrees = 1.5;
      public static final double limeRotMovingToleranceDegrees = 2.0;
      public static final double autoRotationMaxSpeed = 
        Constants.demo.inDemoMode? Constants.demo.rotationScaleFactor
                                 : 0.5;  // don't change - autonomous critical!
      public static final double movingVelocityThresholdFtPerSec = 0.2;  // rotation doesn't count as movement
      public static final double smallNonZeroSpeed = 0.001;  // not enough to move the robot
      public static final double wheelPreRotateSec = 0.5;
      public static final double minAutoRotateSpeed = 0.03;

      public static final double drivePolarDeadband = 0.06;
      public static final double rotatePolarDeadband = 0.5;
      public static final double twistDeadband = 0.08;

      public static final double sideCamDriveScaleFactor = 0.25;
      public static final double sideCamRotationScaleFactor = 0.3;
      public static final double normalRotationScaleFactor = 0.6;

      public static final double disableBreakSec = 2.0;  // don't immediately coast after disabling

      // thresholds at which to engage anti-tipping logic
      public final static class Tip {
          public static final double highVelocityFtPerSec = 6.0; 
          public static final double lowVelocityFtPerSec = 3.0; 
          public static final double highAccFtPerSec2 = 8.0;
          public static final double lowAccFtPerSec2 = 4.0;
          public static final double velAccDiffMaxDeg = 30;
          public static final double highPowerOff = 0.4;
          public static final double lowPowerOff = 0.19;
          public static final double highSpeedSteeringChangeMaxDegrees = 20;
          public static final double velocityHistorySeconds = 0.1;
      }

      public final static class Rotation {
          public static final double kP = 1.2;
          public static final double kD = 6.0;

          public static final double configCLosedLoopRamp = 0.08;
          public static final double minPower = 0.0;  // allow for tighter tolerance
          public static final double maxPower = 0.3;  // reduce gear wear and overshoot
          public static final double countToDegrees = 360.0 / falconEncoderResolution * 12 / 24 * 14 / 72;

          public static final double configVoltageCompSaturation = 11.5;
          public static final boolean enableVoltageCompensation = true;

          public static final boolean statorEnabled = true;
          public static final double statorLimit = 40;
          public static final double statorThreshold = 45;
          public static final double statorTime = 1.0;

          public static final boolean supplyEnabled = true;
          public static final double supplyLimit = 30;
          public static final double supplyThreshold = 35;
          public static final double supplyTime = 0.5;

          public static final double allowableClosedloopError = 0.35 / countToDegrees;

          // values obtained from swerve module zeroing procedure
          public static final double[] CANCoderOffsetDegrees;
          static {
              CANCoderOffsetDegrees = new double[4];
              CANCoderOffsetDegrees[WheelPosition.FRONT_RIGHT.wheelNumber] = 78.311;
              CANCoderOffsetDegrees[WheelPosition.FRONT_LEFT.wheelNumber] = 169.365;
              CANCoderOffsetDegrees[WheelPosition.BACK_RIGHT.wheelNumber] = 106.787;
              CANCoderOffsetDegrees[WheelPosition.BACK_LEFT.wheelNumber] = -104.678;
          }
      }

      public static final class Drive {
          public static final double configOpenLoopRamp = 0.08;

          public static final double voltageCompSaturation = 11.5;
          public static final boolean enableVoltageCompensation = true;

          public static final double brakeModeDeadband = 0.01;

          public static final boolean statorEnabled = true;
          public static final double statorLimit = 40;
          public static final double statorThreshold = 45;
          public static final double statorTime = 1.0;

          public static final boolean supplyEnabled = true;
          public static final double supplyLimit = 40;
          public static final double supplyThreshold = 45;
          public static final double supplyTime = 0.5;

          public static final double wheelDiameter = 4.0;
          public static final double gearRatio = 6.55;
      }

  }

  public static final class ShooterConstants {

      public static final boolean shooterEnabled = true;

      public static final int flywheelLeftID = 14;
      public static final int flywheelRightID = 15;
      public static final double voltageCompSaturation = 11.0;

      public static final double kP = .00025;
      public static final double kI = 0.000001;
      public static final double kD = 0.008;
      public static final double kIz = 200;
      public static final double kFF = 0.000205;
      public static final double kMinRange = 0;   // let flywheel coast down, don't apply power to slow it
      public static final double kMaxRange = 1.0;
      public static final double rampRate = 0.6;  // seconds to go from stopped to full power
      public static final double minVelErrorPreset = 35; // allowable error to shoot (in rpm)
      public static final double minVelErrorLime = 35;
      public static final double speedSettlingPresetSec = 0.1; // don't shoot until speed is stable
      public static final double speedSettlingLimeSec = 0.1;
      public static final double resetSettingDelta = 100;
      public static final double logIntervalDistIn = 5;
  }



  public static final class KickerConstants {
      public static final int kickerID = 16;
      public static final double voltageCompSaturation = 11.0;
      
      public static final double kP = .00025;
      public static final double kI = 0.000001;
      public static final double kD = 0.008;
      public static final double kIz = 200;
      public static final double kFF = 0.000205;
      public static final double kMinRange = 0;   // let flywheel coast down, don't apply power to slow it
      public static final double kMaxRange = 1.0;
      public static final double rampRate = 0.4;  // seconds to go from stopped to full power
      public static final double minVelErrorPreset = 35; // allowable error to shoot (in rpm)
      public static final double minVelErrorLime = 35; // allowable error to shoot (in rpm)
      public static final double speedSettlingPresetSec = 0.1;  // don't shoot until speed is stable
      public static final double speedSettlingLimeSec = 0.1;  // don't shoot until speed is stable
      public static final double minShotSec = 0.2;  // don't stop conveyor when kicker slows down as cargo enters
      public static final double resetSettingDelta = 100;
      public static final int currentLimit = 20;
  }
  
  public static final class HoodConstants {
      public static final int motorID = 17;
      public static final int tolerancePreset = 20;
      public static final int toleranceLime = 20;
      public static final double homingTimeout = 6.0; 

      public static double maxForwardPower = 0.6;    //allows to be changed in demo mode
      public static double maxReversePower = -0.5;   //allows to be changed in demo mode
      public static final double minForwardPower = 0.1;
      public static final double minReversePower = -0.1;
      public static final double homingPower = -0.4;
      public static final double secondHomingPower = -0.2;

      public static final int kPIDLoopIdx = 0;
      public static final int kTimeoutMs = 30;
      public static final boolean kSensorPhase = false;
      public static final boolean kMotorInvert = false;

      public static final double kP = 3.0;
      public static final double kI = 0;
      public static final double kD = 0;
  }

  public static final class IntakeConstants {
      public static final int motorID = 18;

      public static final double kP = .00025;
      public static final double kI = 0.000001;
      public static final double kD = 0.0;
      public static final double kIz = 200;
      public static final double kFF = 0.00033;
      public static final double rampRate = 0.6;  // seconds to go from stopped to full power
      public static final double intakeVelocity = 1350;
      public static final double ejectVelocity = -1350;

      public static final double minRunVel = 100;
      public static final double stallTimeoutSec = 1;

      public static final int currentLimit = 30;
  }

  public static final class ConveyorConstants {
      public static final int motorID = 19;
      public static final double conveyorPower = 0.5;
      public static final int ballSensorPort = 0; // DIO port for ball sensor

      public static final double kickerInjectSec = 0.5;  // time to advance ball into the kicker
      public static final double kickerClearanceSec = kickerInjectSec + 0.2;
      public static final double shooterClearanceSec = kickerClearanceSec + 0.1;
  }

  public static final class WebCams {
      public static final int fps = 10;
      public static final int sideDarkExposure = 10;
      public static final int frontDarkExposure = 2;  // front cam exposure works differently
      public static final int darkBrightness = 5;
      public static final int autoBrightness = 50;
  }

  public static final class LimelightConstants {
      // All distance values in inches
      public static final double limelightAngle = 33;
      public static final double targetHeight = 103;
      public static final double limelightHeight = 38;
  }

  public static final class ClimberConstants {

    public static final int climberLeftID = 20;
    public static final int climberRightID = 21;
    public static final double configVoltageCompSaturation = 11.5;
    public static final boolean enableVoltageCompensation = true;

    // gear ratio of motor to climber revolutions is 245.4545... : 1
    public static final double fullRotation = 245.45454545 * falconEncoderResolution;

    public static final double horizontal = -43615;
    public static final double vertical = 86141;
    public static final double firstEngage = 134673;  // performed manually currently
    public static final double floatingSecondBar= 339500;
    public static final double engageSecondBar= 292500;
    public static final double disengageFirstBar = 180000;
    public static final double floatingThirdBar = floatingSecondBar + fullRotation/2;
    public static final double engageThirdBar = engageSecondBar + fullRotation/2;
    public static final double disengageSecondBar = disengageFirstBar + fullRotation/2;
    public static final double donePos = vertical + fullRotation;
    public static final int positionTolerance = 1000;
    public static final double minRunVel = 250;
    public static final double hookSwingSec = 1.5;
    public static final double overrideTime = 60;
    public static final double poweredDescentAmps = 3; // should always be positive (going fwd)
    
    public static final double fwdOneWayZoneMin = 111700;
    public static final double fwdOneWayZoneMax = 119900; 
    public static final double bwdOneWayZoneMin = 66500;
    public static final double bwdOneWayZoneMax = 73000; 

    public static final boolean statorEnabled = true;
    public static final double statorLimit = 40;
    public static final double statorThreshold = 25;
    public static final double statorTime = 1.0;

    public static final boolean supplyEnabled = true;
    public static final double supplyLimit = 40;
    public static final double supplyThreshold = 25;
    public static final double supplyTime = 1.0;

    public static final double kPUnloaded = 0.025;
    public static final double kPLoaded = 0.075;
    public static final double kDUnloaded = 0;
    public static final double kDLoaded = 0;
    public static final double kMinRange = 0.07;
    public static final double kMaxRange = 0.6;
    public static final double rampRate = 0.6;

    public static final double manualDeadband = 0.1;
  }
}
