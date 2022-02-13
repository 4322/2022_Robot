// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShuffleboardTab tab;

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;

  private RelativeEncoder flywheelEncoder;
  private SparkMaxPIDController flywheelPID;

  public Shooter() {
    flywheelOne = new CANSparkMax(Constants.ShooterConstants.flywheelOneID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.ShooterConstants.flywheelTwoID, MotorType.kBrushless);

    flywheelOne.restoreFactoryDefaults();
    flywheelOne.setInverted(true);
    flywheelTwo.restoreFactoryDefaults();
    flywheelTwo.follow(flywheelOne, true);
    flywheelOne.setIdleMode(IdleMode.kCoast);
    flywheelTwo.setIdleMode(IdleMode.kCoast);
    flywheelOne.setClosedLoopRampRate(Constants.ShooterConstants.rampRate);  // don't eject the shooter

    flywheelEncoder = flywheelOne.getEncoder();
    flywheelPID = flywheelOne.getPIDController();

    flywheelPID.setP(Constants.ShooterConstants.kP);
    flywheelPID.setI(Constants.ShooterConstants.kI);
    flywheelPID.setD(Constants.ShooterConstants.kD);
    flywheelPID.setIZone(Constants.ShooterConstants.kIz);
    flywheelPID.setFF(Constants.ShooterConstants.kFF);
    flywheelPID.setOutputRange(Constants.ShooterConstants.kMinRange, Constants.ShooterConstants.kMaxRange);
  }

  @Override
  public void periodic() {
    
  }

}
