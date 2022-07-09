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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;

  private double target;

  // to be enabled if debug mode is on
  private ShuffleboardTab tab;
  
  private NetworkTableEntry power;
  private NetworkTableEntry currentRPM;
  private NetworkTableEntry targetRPM;
  private NetworkTableEntry override;

  private Timer modeTimer = new Timer();

  public Climber() {
    if (Constants.climberEnabled) {
      climberLeft = new WPI_TalonFX(ClimberConstants.climberLeftID);
      climberRight = new WPI_TalonFX(ClimberConstants.climberRightID);
      RobotContainer.staggerTalonStatusFrames(climberLeft);
      RobotContainer.staggerTalonStatusFrames(climberRight);
    }
      modeTimer.start();
 
    }

  public void init() {
    if (Constants.climberEnabled) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      climberLeft.configAllSettings(config);
      climberLeft.setInverted(false);
      climberRight.configAllSettings(config);
      climberRight.follow(climberLeft);
      climberLeft.setNeutralMode(NeutralMode.Coast);
      climberRight.setNeutralMode(NeutralMode.Coast);
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);  // don't eject the shooter 
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate);    // for PID tuning
      climberLeft.configVoltageCompSaturation(ClimberConstants.configVoltageCompSaturation);
		  climberLeft.enableVoltageCompensation(ClimberConstants.enableVoltageCompensation);


      private void configRotation(WPI_TalonFX talon) {

        TalonFXConfiguration configClimber = new TalonFXConfiguration();    
        config.slot0.kP = ClimberConstants.kP;
        config.slot0.kD = ClimberConstants.kD;
        config.nominalOutputForward = ClimberConstants.kMinRange;
		    config.nominalOutputReverse = -ClimberConstants.kMinRange;
		    config.peakOutputForward = ClimberConstants.kMaxRange;
		    config.peakOutputReverse = -ClimberConstants.kMaxRange;
      }

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
    
      // DEBUG
      if (Constants.debug) {
        tab = Shuffleboard.getTab("Climber");
      
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
  public enum ClimberMode {
    stopped(0),
    started(1),
    atSpeed(2),
    stableAtSpeed(3),
    stopping(4);

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
  }


  public double getSpeed() {
    if (Constants.climberEnabled) {
      return climberEncoder.getVelocity();
    } else {
      return -1;
    }
  }

  public boolean isRunning() {
    return (climberMode != ClimberMode.stopped) && (climberMode != ClimberMode.stopping);
  }
  
  public void stop() {
    climberMode = ClimberMode.stopping;
  }

  private double getPosition() {
    return climberLeft.getSelectedSensorPosition(0);
  }
}