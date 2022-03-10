// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private WPI_TalonSRX conveyor;
  private DigitalInput ballSensor;

  public Conveyor() {
    if (Constants.conveyorEnabled) {
      conveyor = new WPI_TalonSRX(ConveyorConstants.motorID);

       // increase status reporting periods to reduce CAN bus utilization
      conveyor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
      conveyor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    }
 }

  public void init() {
    if (Constants.conveyorEnabled) {
      conveyor.configFactoryDefault();
      setCoastMode();  // Allow manual movement until enabled

      if (Constants.ballSensorEnabled) {
        ballSensor = new DigitalInput(ConveyorConstants.ballSensorPort);
      }
    }
  }

  @Override
  public void periodic() {
  }

  public void enableConveyor() {
    if (Constants.conveyorEnabled) {
      conveyor.set(ConveyorConstants.conveyorPower);
    }
  }

  public boolean getSensedBall() {
    if (Constants.ballSensorEnabled) {
      return !ballSensor.get();
    } else {
      return false;
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