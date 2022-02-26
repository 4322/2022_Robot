// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax flywheelLeft;
  private CANSparkMax flywheelRight;

  private RelativeEncoder flywheelEncoder;
  private SparkMaxPIDController flywheelPID;

  // to be enabled if debug mode is on
  private ShuffleboardTab tab;
  
  private NetworkTableEntry power;
  private NetworkTableEntry currentRPM;
  private NetworkTableEntry targetRPM;

  public Shooter() {
    flywheelLeft = new CANSparkMax(ShooterConstants.flywheelLeftID, MotorType.kBrushless);
    flywheelRight = new CANSparkMax(ShooterConstants.flywheelRightID, MotorType.kBrushless);

    flywheelLeft.restoreFactoryDefaults();
    flywheelLeft.setInverted(true);
    flywheelRight.restoreFactoryDefaults();
    flywheelRight.follow(flywheelLeft, true);
    flywheelLeft.setIdleMode(IdleMode.kCoast);
    flywheelRight.setIdleMode(IdleMode.kCoast);
    flywheelLeft.setClosedLoopRampRate(ShooterConstants.rampRate);  // don't eject the shooter

    flywheelEncoder = flywheelLeft.getEncoder();
    flywheelPID = flywheelLeft.getPIDController();

    flywheelPID.setP(ShooterConstants.kP);
    flywheelPID.setI(ShooterConstants.kI);
    flywheelPID.setD(ShooterConstants.kD);
    flywheelPID.setIZone(ShooterConstants.kIz);
    flywheelPID.setFF(ShooterConstants.kFF);
    flywheelPID.setOutputRange(ShooterConstants.kMinRange, ShooterConstants.kMaxRange);

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
    }
  }

  @Override
  public void periodic() {
    if (Constants.debug) {
      power.setDouble(flywheelLeft.getAppliedOutput());
      currentRPM.setDouble(getSpeed());
    }
  }

  public void setSpeed(double rpm) {
    flywheelPID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    if (Constants.debug) {
      targetRPM.setDouble(rpm);
    }
  }

  public double getSpeed() {
    return flywheelEncoder.getVelocity();
  }

  // don't let balls get stuck in the shooter
  public boolean isAbleToEject() {
    return getSpeed() >= ShooterConstants.minEjectVel;
  }
  
  public void stopShooter() {
    flywheelLeft.stopMotor();
  }
}
