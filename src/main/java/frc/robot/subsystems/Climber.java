// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

public class Climber extends SubsystemBase {

  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;

  private double target;

  private RelativeEncoder climberEncoder;
  private SparkMaxPIDController climberPID;

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
      climberLeft.configFactoryDefault();
      climberLeft.setInverted(false);
      climberRight.configFactoryDefault();
      climberRight.follow(climberLeft);
      climberLeft.setNeutralMode(NeutralMode.Coast);
      climberRight.setNeutralMode(NeutralMode.Coast);
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);  // don't eject the shooter
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate);    // for PID tuning
      climberLeft.enableVoltageCompensation(ClimberConstants.voltageCompSaturation);

      climberEncoder = climberLeft.getEncoder();
      climberPID = climberLeft.getPIDController();

      climberPID.setP(ClimberConstants.kP);
      climberPID.setI(ClimberConstants.kI);
      climberPID.setD(ClimberConstants.kD);
      climberPID.setIZone(ClimberConstants.kIz);
      climberPID.setFF(ClimberConstants.kFF);
      climberPID.setOutputRange(ClimberConstants.kMinRange, ClimberConstants.kMaxRange);

      climberLeft.burnFlash();
      climberRight.burnFlash();

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

  // don't let balls get stuck in the shooter
  public boolean isAtSpeed() {
    return climberMode == ClimberMode.stableAtSpeed;
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
