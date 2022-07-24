/* 
Code review comment:
Determine how you want to abort a climb. We could only continue while the start button is held down, 
but we can't restart the entire sequence if the operator accidentally releases the button because the 
climber would no longer be in initial position. The other buttons aren't needed once we start climbing, 
so we could redefine one of them as the abort. We will figure out how to handle automatic feedback, 
such as a motor stall or robot tilt, with Torsten on Wednesday.
*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  // to be enabled if debug mode is on
  private ShuffleboardTab tab;
  private NetworkTableEntry positionDisplay;
  private NetworkTableEntry targetDisplay;

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
      config.slot0.kP = ClimberConstants.kP;
      config.slot0.kD = ClimberConstants.kD;
      config.slot0.allowableClosedloopError = ClimberConstants.positionTolerance;
      config.nominalOutputForward = ClimberConstants.kMinRange;
      config.nominalOutputReverse = -ClimberConstants.kMinRange;
      config.peakOutputForward = ClimberConstants.kMaxRange;
      config.peakOutputReverse = -ClimberConstants.kMaxRange;
      climberLeft.configAllSettings(config);
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate); // for PID tuning
      configCurrentLimit(climberLeft);
      configCurrentLimit(climberRight);
      climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,  // rapid updates for follower
        RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
      climberLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,  // for position error feedback
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
    return (Math.abs(climberLeft.getClosedLoopError()) <= ClimberConstants.positionTolerance);
  }

  public void moveToPosition(double pos) {
    if (Constants.climberEnabled) {
      climberLeft.set(ControlMode.Position, pos);
      if (Constants.debug) {
        targetDisplay.setDouble(pos);
      }
    }
  }

  public void setCurrentPosition(double pos) {
    if (Constants.hoodEnabled) {
      climberLeft.setSelectedSensorPosition(pos);
    }
  }

  public void setCoastMode() {
    climberLeft.setNeutralMode(NeutralMode.Coast);
    climberRight.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);
  }
}