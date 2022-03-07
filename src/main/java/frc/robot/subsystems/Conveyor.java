// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private WPI_TalonSRX conveyor;
  private Ultrasonic ultrasonic;

  public Conveyor() {
    if (Constants.conveyorEnabled) {
      conveyor = new WPI_TalonSRX(ConveyorConstants.motorID);

      conveyor.configFactoryDefault();
      setCoastMode();  // Allow manual movement until enabled

      if (Constants.ultrasonicEnabled) {
        ultrasonic = new Ultrasonic(1, 2);
      }
    }
  }

  @Override
  public void periodic() {

  }

  public void enableConveyor() {
    if (Constants.conveyorEnabled && (ultrasonic.getRangeInches() <= ConveyorConstants.minBallDistIn)) {
      conveyor.set(ConveyorConstants.conveyorPower);
    }
  }

  public void enableConveyorOverride() {
    if (Constants.conveyorEnabled) {
      conveyor.set(ConveyorConstants.conveyorPower);
    }
  }

  public void stop() {
    if (Constants.conveyorEnabled) {
      conveyor.stopMotor();
    }
  }

  public void setCoastMode() {
    if (Constants.hoodEnabled) {
      conveyor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setBrakeMode() {
    if (Constants.hoodEnabled) {
      conveyor.setNeutralMode(NeutralMode.Brake);
    }
  }
}