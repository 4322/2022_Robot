// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private CANSparkMax conveyor;
  private Ultrasonic ultrasonic;

  public Conveyor() {
    if (Constants.conveyorEnabled) {
      conveyor = new CANSparkMax(ConveyorConstants.conveyorID, MotorType.kBrushless);
      conveyor.restoreFactoryDefaults();
      conveyor.setInverted(true);
      conveyor.setIdleMode(IdleMode.kBrake);   // don't let balls partially fall into the shooter
      conveyor.burnFlash();

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

  public void disableConveyor() {
    if (Constants.conveyorEnabled) {
      conveyor.stopMotor();
    }
  }
}