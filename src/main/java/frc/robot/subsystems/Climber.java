// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;

  private Double currentTarget = null;

  // to be enabled if debug mode is on
  private ShuffleboardTab tab;
  private NetworkTableEntry positionDisplay;
  private NetworkTableEntry targetDisplay;

  // only unlock the climber if climber has been set to vertical position or
  // manually unlocked (stopped)
  private boolean climbLocked = true;

  public enum climbMode {
    loaded,
    unloaded
  }

  private enum lockedDir {
    forward,
    backward,
    none
  }

  private lockedDir currentLockedDir = lockedDir.none;
  private double lastPos = 0;

  public Climber() {
    if (Constants.climberEnabled) {
      climberLeft = new WPI_TalonFX(ClimberConstants.climberLeftID);
      climberRight = new WPI_TalonFX(ClimberConstants.climberRightID);
      RobotContainer.staggerTalonStatusFrames(climberLeft);
      RobotContainer.staggerTalonStatusFrames(climberRight);
    }
  }

  public void init() {
    if (Constants.climberEnabled) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.slot0.kP = ClimberConstants.kPUnloaded;
      config.slot0.kD = ClimberConstants.kDUnloaded;
      config.slot0.allowableClosedloopError = ClimberConstants.positionTolerance;
      config.slot1.kP = ClimberConstants.kPLoaded;
      config.slot1.kD = ClimberConstants.kDLoaded;
      config.slot1.allowableClosedloopError = ClimberConstants.positionTolerance;
      config.nominalOutputForward = ClimberConstants.kMinRange;
      config.nominalOutputReverse = -ClimberConstants.kMinRange;
      config.peakOutputForward = ClimberConstants.kMaxRange;
      config.peakOutputReverse = -ClimberConstants.kMaxRange;
      climberLeft.configAllSettings(config);
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate); // for PID tuning
      configCurrentLimit(climberLeft);
      configCurrentLimit(climberRight);
      climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, // rapid updates for follower
          RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
      climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, // for position error feedback
          RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
      climberRight.follow(climberLeft);
      climberRight.setInverted(InvertType.OpposeMaster);
      climberLeft.setNeutralMode(NeutralMode.Brake);
      climberRight.setNeutralMode(NeutralMode.Brake);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Climber");
        positionDisplay = tab.add("Position", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        targetDisplay = tab.add("Target", 0)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
      }
    }
  }

  private void configCurrentLimit(WPI_TalonFX talon) {
    talon.configVoltageCompSaturation(ClimberConstants.configVoltageCompSaturation);
    talon.enableVoltageCompensation(ClimberConstants.enableVoltageCompensation);
    talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
        ClimberConstants.statorEnabled,
        ClimberConstants.statorLimit,
        ClimberConstants.statorThreshold,
        ClimberConstants.statorTime));
    talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        ClimberConstants.supplyEnabled,
        ClimberConstants.supplyLimit,
        ClimberConstants.supplyThreshold,
        ClimberConstants.supplyTime));
  }

  @Override
  public void periodic() {
    if (Constants.climberEnabled) {
      updateLockedDir();
      if (Constants.debug) {
        positionDisplay.setDouble(getPosition());
      }
    }
  }

  public double getSpeed() {
    if (Constants.climberEnabled) {
      return climberLeft.getSelectedSensorVelocity();
    } else {
      return -1;
    }
  }

  private void updateLockedDir() {

    double pos = getPosition();

    // number of encoder ticks from starting position, half rotations to account for two climber arms
    double posRelativeStarting = pos % (ClimberConstants.fullRotation/2);
  
    double fwdMinZone = ClimberConstants.fwdOneWayZoneMin;
    double fwdMaxZone = ClimberConstants.fwdOneWayZoneMax;
    double bwdMinZone = ClimberConstants.bwdOneWayZoneMin;
    double bwdMaxZone = ClimberConstants.bwdOneWayZoneMax;
    //forward when top of climber rotates to the front of robot
    //backward when top of climber rotates to the back of robot

    boolean inFwdZone = (posRelativeStarting > fwdMinZone) && (posRelativeStarting < fwdMaxZone);
    boolean inBwdZone = (posRelativeStarting > bwdMinZone) && (posRelativeStarting < bwdMaxZone);

    if (currentLockedDir == lockedDir.none) {
      if (inFwdZone) {
        if (Math.abs(posRelativeStarting - fwdMaxZone) < Math.abs(posRelativeStarting - fwdMinZone)) {
          currentLockedDir = lockedDir.backward;
        } else {
          currentLockedDir = lockedDir.forward;
        }
      } else if (inBwdZone) {
        if (Math.abs(posRelativeStarting - bwdMaxZone) < Math.abs(posRelativeStarting - bwdMinZone)) {
          currentLockedDir = lockedDir.forward;
        } else {
          currentLockedDir = lockedDir.backward;
        }
      }
      else {
        currentLockedDir = lockedDir.none;
      }
    } else if (!inFwdZone && !inBwdZone) {
      currentLockedDir = lockedDir.none;
    } else {
      return;
    }

    // update last position
    lastPos = pos;

  }

  public void stop() {
    climberLeft.stopMotor();
  }

  private double getPosition() {
    if (Constants.climberEnabled) {
      return climberLeft.getSelectedSensorPosition(0);
    } else {
      return -1;
    }
  }

  public boolean isAtTarget() {
    if (!Constants.climberEnabled) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= ClimberConstants.positionTolerance);
  }

  public void moveToPosition(double pos, climbMode mode) {
    if (Constants.climberEnabled) {
      switch (mode) {
        case unloaded:
          climberLeft.selectProfileSlot(0, 0);
          break;
        case loaded:
          climberLeft.selectProfileSlot(1, 0);
          break;
      }
      currentTarget = pos;
      if (Constants.debug) {
        targetDisplay.setDouble(pos);
      }
    }
  }

  public void setCurrentPosition(double pos) {
    if (Constants.climberEnabled) {
      climberLeft.setSelectedSensorPosition(pos);
    }
  }

  public void setClimberSpeed(double speed) {
    if (Constants.climberEnabled) {
      if (currentLockedDir == lockedDir.none) {
        climberLeft.set(speed);
      } else if ((currentLockedDir == lockedDir.forward) && (speed >= 0)) {
        climberLeft.set(speed);
      } else if ((currentLockedDir == lockedDir.backward) && (speed <= 0)) {
        climberLeft.set(speed);
      } else {
        stop();
      }
    }
  }

  public void unlockClimb() {
    climbLocked = false;
  }

  public void lockClimb() {
    climbLocked = true;
  }

  public boolean isClimbLocked() {
    return climbLocked;
  }

  public void setCoastMode() {
    if (Constants.climberEnabled) {
      climberLeft.setNeutralMode(NeutralMode.Coast);
      climberRight.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setBrakeMode() {
    if (Constants.climberEnabled) {
      climberLeft.setNeutralMode(NeutralMode.Brake);
      climberRight.setNeutralMode(NeutralMode.Brake);
    }
  }
}