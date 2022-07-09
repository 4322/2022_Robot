// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
      config.nominalOutputForward = ClimberConstants.kMinRange;
      config.nominalOutputReverse = -ClimberConstants.kMinRange;
      config.peakOutputForward = ClimberConstants.kMaxRange;
      config.peakOutputReverse = -ClimberConstants.kMaxRange;
      climberLeft.configAllSettings(config);
      configCurrentLimit(climberLeft);
      climberLeft.setInverted(false);
      configCurrentLimit(climberRight);
      climberRight.follow(climberLeft);
      climberLeft.setNeutralMode(NeutralMode.Coast);
      climberRight.setNeutralMode(NeutralMode.Coast);
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);  // don't eject the shooter 
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate);    // for PID tuning
      climberLeft.configVoltageCompSaturation(ClimberConstants.configVoltageCompSaturation);
		  climberLeft.enableVoltageCompensation(ClimberConstants.enableVoltageCompensation);
      if (Constants.debug) {
        tab = Shuffleboard.getTab("Climber");
        positionDisplay = tab.add("Position", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
      }
    }
  }

      private void configCurrentLimit(WPI_TalonFX talon) {
      
      climberLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			  ClimberConstants.statorEnabled, 
			  ClimberConstants.statorLimit, 
			  ClimberConstants.statorThreshold, 
			  ClimberConstants.statorTime));
		  climberLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			  ClimberConstants.supplyEnabled, 
			  ClimberConstants.supplyLimit, 
			  ClimberConstants.supplyThreshold, 
			  ClimberConstants.supplyTime));

      climberRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			  ClimberConstants.statorEnabled, 
			  ClimberConstants.statorLimit, 
			  ClimberConstants.statorThreshold, 
			  ClimberConstants.statorTime));
		  climberRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			  ClimberConstants.supplyEnabled, 
			  ClimberConstants.supplyLimit, 
			  ClimberConstants.supplyThreshold, 
			  ClimberConstants.supplyTime));
    }

  public enum ClimberMode {
    stopped(0),
    forward1(1),
    backward1(2),
    forward2(3),
    done(4);

    private int value;

    ClimberMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private ClimberMode climberMode = ClimberMode.stopped;

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

  public boolean isRunning() {
    return (climberMode != ClimberMode.stopped) && (climberMode != ClimberMode.done);
  }
  
  public void stop() {
    climberMode = ClimberMode.done;
  }

  private double getPosition() {
    return climberLeft.getSelectedSensorPosition(0);
  }
}