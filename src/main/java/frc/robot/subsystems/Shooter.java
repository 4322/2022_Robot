// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax flywheelLeft;
  private CANSparkMax flywheelRight;

  private double target;

  private RelativeEncoder flywheelEncoder;
  private SparkMaxPIDController flywheelPID;

  // to be enabled if debug mode is on
  private ShuffleboardTab tab;
  
  private NetworkTableEntry power;
  private NetworkTableEntry currentRPM;
  private NetworkTableEntry targetRPM;
  private NetworkTableEntry override;

  private Timer modeTimer = new Timer();

  public Shooter() {
    if (Constants.shooterEnabled) {
      flywheelLeft = new CANSparkMax(ShooterConstants.flywheelLeftID, MotorType.kBrushless);
      flywheelRight = new CANSparkMax(ShooterConstants.flywheelRightID, MotorType.kBrushless);

      modeTimer.start();

      // increase status reporting periods to reduce CAN bus utilization
      flywheelLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 
        RobotContainer.nextShuffleboardStatusPeriodMs());  
      flywheelLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 
        RobotContainer.nextFastStatusPeriodMs());  // to detect when we can shoot
      flywheelLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());  
      flywheelRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 
        RobotContainer.nextSlowStatusPeriodMs());
      flywheelRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());
      flywheelRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());  
    }
  }

  public void init() {
    if (Constants.shooterEnabled) {
      flywheelLeft.restoreFactoryDefaults();
      flywheelLeft.setInverted(false);
      flywheelRight.restoreFactoryDefaults();
      flywheelRight.follow(flywheelLeft, true);
      flywheelLeft.setIdleMode(IdleMode.kCoast);
      flywheelRight.setIdleMode(IdleMode.kCoast);
      flywheelLeft.setClosedLoopRampRate(ShooterConstants.rampRate);  // don't eject the shooter
      flywheelLeft.setOpenLoopRampRate(ShooterConstants.rampRate);    // for PID tuning
      flywheelLeft.enableVoltageCompensation(ShooterConstants.voltageCompSaturation);

      flywheelEncoder = flywheelLeft.getEncoder();
      flywheelPID = flywheelLeft.getPIDController();

      flywheelPID.setP(ShooterConstants.kP);
      flywheelPID.setI(ShooterConstants.kI);
      flywheelPID.setD(ShooterConstants.kD);
      flywheelPID.setIZone(ShooterConstants.kIz);
      flywheelPID.setFF(ShooterConstants.kFF);
      flywheelPID.setOutputRange(ShooterConstants.kMinRange, ShooterConstants.kMaxRange);

      flywheelLeft.burnFlash();
      flywheelRight.burnFlash();

      // DEBUG
      if (Constants.debug) {
        tab = Shuffleboard.getTab("Shooter");
      
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

  // no need for shooting state as in Kicker because the conveyor 
  // isn't needed once cargo enters the shooter
  public enum ShooterMode {
    stopped(0),
    started(1),
    atSpeed(2),
    stableAtSpeed(3);

    private int value;

    ShooterMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private ShooterMode shooterMode = ShooterMode.stopped;

  @Override
  public void periodic() {
    if (Constants.shooterEnabled) {
      if (Constants.debug) {
        if (override.getBoolean(false) && (target != targetRPM.getDouble(0))) {
          setSpeed(targetRPM.getDouble(0));
        }
        power.setDouble(flywheelLeft.getAppliedOutput());
        currentRPM.setDouble(getSpeed());
      }
      boolean isAtSpeed = Math.abs(target - getSpeed()) <= ShooterConstants.minVelError;
      switch (shooterMode) {
        case stopped:
          break;
        case started:
          if (isAtSpeed) {
            shooterMode = ShooterMode.atSpeed;
            modeTimer.reset();
          }
          break;
        case atSpeed:
          if (isAtSpeed && modeTimer.hasElapsed(ShooterConstants.speedSettlingSec)) {
            shooterMode = ShooterMode.stableAtSpeed;
          } else if (!isAtSpeed) {
            shooterMode = ShooterMode.started;  // restart settling timer
          }
          break;
        case stableAtSpeed:
          if (!isAtSpeed) {
            shooterMode = ShooterMode.started;  // restart settling timer
          }
          break;
      }
    }
  }

  public void setSpeed(double rpm) {
    if (Constants.shooterEnabled) {
      flywheelPID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
      target = rpm;
      shooterMode = ShooterMode.started;
      if (Constants.debug) {
        targetRPM.setDouble(rpm);
      }
    }
  }

  public double getSpeed() {
    if (Constants.shooterEnabled) {
      return flywheelEncoder.getVelocity();
    } else {
      return -1;
    }
  }

  // don't let balls get stuck in the shooter
  public boolean isAbleToEject() {
    return shooterMode == ShooterMode.stableAtSpeed;
  }
  
  public void stop() {
    if (Constants.shooterEnabled) {
      flywheelLeft.stopMotor();
      shooterMode = ShooterMode.stopped;
    }
  }
}
