// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private WPI_TalonSRX conveyor;
  private DigitalInput ballSensor;
  private boolean manualAdvanceCargoActive;
  private boolean autoAdvanceCargoActive;
  private Timer shotTimer = new Timer();

  private static Conveyor singleton;

  public enum ConveyorMode {
    stopped(0),
    advancingCargo(1),
    loaded(2),
    shooting(3),
    stopping(4);

    private int value;

    ConveyorMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private ConveyorMode conveyorMode = ConveyorMode.stopped;

  private Conveyor() {
    singleton = this;

    if (Constants.conveyorEnabled) {
      conveyor = new WPI_TalonSRX(ConveyorConstants.motorID);
      RobotContainer.staggerTalonStatusFrames(conveyor);
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
    if (Constants.conveyorEnabled) {
      switch (conveyorMode) {
        case stopped:
          break;
        case advancingCargo:
          if (getSensedBall()) {
            conveyorMode = ConveyorMode.loaded;
            conveyor.stopMotor();
          }
          break;
        case loaded:
          if(!getSensedBall()) {
            DriverStation.reportError("Ball has slipped past button", false);
            conveyorMode = ConveyorMode.stopped;
          }
          break;
        case shooting:
          if (shotTimer.hasElapsed(Constants.ConveyorConstants.kickerInjectSec)) {
            conveyorMode = ConveyorMode.advancingCargo;
          }
          break;
        case stopping:
          if (shotTimer.hasElapsed(Constants.ConveyorConstants.kickerInjectSec)) {
            if (autoAdvanceCargoActive || manualAdvanceCargoActive) {
              conveyorMode = ConveyorMode.advancingCargo;
            } else {
              conveyorMode = ConveyorMode.stopped;
              conveyor.stopMotor();
            }
          }
          break;
      }
    }
  }

  public boolean canKickerStop() {
    return (conveyorMode != ConveyorMode.shooting) || 
            shotTimer.hasElapsed(Constants.ConveyorConstants.kickerClearanceSec);
  }

  public boolean canShooterStop() {
    return (conveyorMode != ConveyorMode.shooting) || 
            shotTimer.hasElapsed(Constants.ConveyorConstants.shooterClearanceSec);
  }

  private void start() {
    conveyor.set(ConveyorConstants.conveyorPower);
  }

  private boolean getSensedBall() {
    if (Constants.ballSensorEnabled) {
      return !ballSensor.get();
    } else {
      return false;
    }
  }

  public boolean isLoaded() {
    return conveyorMode == ConveyorMode.loaded;
  }

  public void shoot() {
    if (Constants.conveyorEnabled){
      if (conveyorMode == ConveyorMode.loaded) {
        conveyorMode = ConveyorMode.shooting;
        start();
        shotTimer.reset();
        shotTimer.start();
        Intake.getSingleton().autoIntake();
        autoAdvanceCargoActive = true;
      }
    }
  }

  public void autoAdvanceCargo() {
    if (Constants.conveyorEnabled) {
      if (conveyorMode == ConveyorMode.stopped) {
        conveyorMode = ConveyorMode.advancingCargo;
        start();
      }
      Intake.getSingleton().autoIntake();
      autoAdvanceCargoActive = true;
    }
  }

  public void autoStop() {
    if (Constants.conveyorEnabled) {
      if (!manualAdvanceCargoActive) {
        if (conveyorMode == ConveyorMode.shooting) {
          conveyorMode = ConveyorMode.stopping;
        } else if (conveyorMode == ConveyorMode.advancingCargo) {
          conveyor.stopMotor();
          conveyorMode = ConveyorMode.stopped;
        }
      }
      Intake.getSingleton().autoStop();
      autoAdvanceCargoActive = false;
    }
  }

  // Can be called by one command only without requirements to continue
  // shooting during manual intake.
  public void manualAdvanceCargo() {
    if (Constants.conveyorEnabled) {
      if (conveyorMode == ConveyorMode.stopped) {
        conveyorMode = ConveyorMode.advancingCargo;
        start();
      }
      Intake.getSingleton().manualIntake();
      manualAdvanceCargoActive = true;
    }
  }

  // Can be called by one command only without requirements to continue
  // shooting during manual intake.
  public void manualStop() {
    if (Constants.conveyorEnabled) {
      if (!autoAdvanceCargoActive) {
        if (conveyorMode == ConveyorMode.shooting) {
          conveyorMode = ConveyorMode.stopping;
        } else if (conveyorMode == ConveyorMode.advancingCargo) {
          conveyor.stopMotor();
          conveyorMode = ConveyorMode.stopped;
        }
      }
      Intake.getSingleton().manualStop();
      manualAdvanceCargoActive = false;
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

  public static Conveyor getSingleton() {

    if (singleton == null) {
        singleton = new Conveyor();
    }

    return singleton;
  }
}