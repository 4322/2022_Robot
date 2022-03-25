package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.KickerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Kicker extends SubsystemBase {

  private CANSparkMax kicker;
  private RelativeEncoder kickerEncoder;
  private SparkMaxPIDController kickerPID;
  private double target;

  private Timer modeTimer = new Timer();

  private ShuffleboardTab tab;
  
  private NetworkTableEntry power;
  private NetworkTableEntry currentRPM;
  private NetworkTableEntry targetRPM;
  private NetworkTableEntry override;

  public Kicker() {
    if (Constants.kickerEnabled) {
      kicker = new CANSparkMax(KickerConstants.kickerID, MotorType.kBrushless);
      modeTimer.start();

      // increase status reporting periods to reduce CAN bus utilization
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 
        RobotContainer.nextSlowStatusPeriodMs());
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 
        RobotContainer.nextFastStatusPeriodMs());  // to detect when we can shoot
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs()); 
    } 
  }

  public void init() {
    if (Constants.kickerEnabled) {
      kicker.restoreFactoryDefaults();
      kicker.setOpenLoopRampRate(KickerConstants.rampRate);
      kicker.enableVoltageCompensation(KickerConstants.voltageCompSaturation);
      kicker.setInverted(false);
      kickerEncoder = kicker.getEncoder();

      kickerPID = kicker.getPIDController();
      kickerPID.setP(KickerConstants.kP);
      kickerPID.setI(KickerConstants.kI);
      kickerPID.setD(KickerConstants.kD);
      kickerPID.setIZone(KickerConstants.kIz);
      kickerPID.setFF(KickerConstants.kFF);  

      kicker.burnFlash();
      setCoastMode();  // Allow manual movement until enabled

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Kicker");
      
        power =
          tab.add("Power", 0)
          .withPosition(0,0)
          .withSize(1,1)
          .getEntry();

        currentRPM =
          tab.add("Current RPM", 0)
          .withPosition(1,0)
          .withSize(1,1)
          .getEntry();

        targetRPM =
          tab.add("Target RPM", 0)
          .withPosition(2,0)
          .withSize(1,1)
          .getEntry();

        override = 
          tab.add("Override", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(0,1)
          .withSize(1,1)
          .getEntry();
      }
    }
  }

  public enum KickerMode {
    stopped(0),
    started(1),
    atSpeed(2),
    stableAtSpeed(3),
    shooting(4);

    private int value;

    KickerMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private KickerMode kickerMode = KickerMode.stopped;

  public void stop() {
    if (Constants.kickerEnabled) {
      kicker.stopMotor();
      kickerMode = KickerMode.stopped;
    }
  }

  @Override
  public void periodic() {
    if (Constants.kickerEnabled) {
      if (Constants.debug) {
        if (override.getBoolean(false) && (target != targetRPM.getDouble(0))) {
          setSpeed(targetRPM.getDouble(0));
        }
        power.setDouble(kicker.getAppliedOutput());
        currentRPM.setDouble(getSpeed());
      }
      boolean isAtSpeed = Math.abs(target - getSpeed()) <= KickerConstants.minVelError;
      switch (kickerMode) {
        case stopped:
          break;
        case started:
          if (isAtSpeed) {
            kickerMode = KickerMode.atSpeed;
            modeTimer.reset();
          }
          break;
        case atSpeed:
          if (isAtSpeed && modeTimer.hasElapsed(KickerConstants.speedSettlingSec)) {
            kickerMode = KickerMode.stableAtSpeed;
          } else if (!isAtSpeed) {
            kickerMode = KickerMode.started;  // restart settling timer
          }
          break;
        case stableAtSpeed:
          if (!isAtSpeed) {
            kickerMode = KickerMode.shooting;  // don't stop conveyor as cargo enters kicker
            modeTimer.reset();
          }
          break;
        case shooting:
          if (modeTimer.hasElapsed(KickerConstants.minShotSec)) {
            kickerMode = KickerMode.started;  // cargo is through kicker, allow conveyor to be stopped
          }
      }
    }
  }

  public void setSpeed(double rpm) {
    if (Constants.kickerEnabled) {
      modeTimer.reset();
      kickerPID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
      target = rpm;
      kickerMode = KickerMode.started;
      if (Constants.debug) {
        targetRPM.setDouble(rpm);
      }
    }
  }

  public double getSpeed() {
    if (Constants.kickerEnabled) {
      return kickerEncoder.getVelocity();
    } else {
      return -1;
    }
  }

  // don't let balls get stuck in the kicker
  public boolean isAbleToEject() {
    return (kickerMode == KickerMode.stableAtSpeed) || (kickerMode == KickerMode.shooting);
  }
  
  public void setCoastMode() {
    if (Constants.kickerEnabled) {
      kicker.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.kickerEnabled) {
      kicker.setIdleMode(IdleMode.kBrake);
    }
  }
}
