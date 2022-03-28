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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private WPI_TalonSRX conveyor;
  private DigitalInput ballSensor;
  private Timer shotTimer = new Timer();

  private static Conveyor singleton;

  public enum ConveyorMode {
    stopped(0),
    intaking(1),
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
        case intaking:
          if (getSensedBall()) {
            conveyorMode = ConveyorMode.loaded;
            conveyor.stopMotor();
          }
          break;
        case loaded:
          break;
        case shooting:
          if (shotTimer.hasElapsed(Constants.ConveyorConstants.kickerInjectSec)) {
            conveyorMode = ConveyorMode.intaking;
          }
          break;
        case stopping:
          if (shotTimer.hasElapsed(Constants.ConveyorConstants.kickerInjectSec)) {
            conveyorMode = ConveyorMode.stopped;
            conveyor.stopMotor();
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

  public boolean getSensedBall() {
    if (Constants.ballSensorEnabled) {
      return !ballSensor.get();
    } else {
      return false;
    }
  }

  public void shoot() {
    if (Constants.conveyorEnabled) {
      if (conveyorMode == ConveyorMode.loaded) {
        conveyorMode = ConveyorMode.shooting;
        start();
        Intake.getSingleton().autoIntake();
        shotTimer.reset();
        shotTimer.start();
      } else if (conveyorMode == ConveyorMode.stopped) {
        conveyorMode = ConveyorMode.intaking;
        start();
        Intake.getSingleton().autoIntake();
      }
    }
  }

  public void intake() {
    if (Constants.conveyorEnabled) {
      if (conveyorMode == ConveyorMode.stopped) {
        conveyorMode = ConveyorMode.intaking;
        start();
        Intake.getSingleton().autoIntake();
      }
    }
  }

  public void stop() {
    if (Constants.conveyorEnabled) {
      Intake.getSingleton().autoStop();
      if (conveyorMode == ConveyorMode.shooting) {
        conveyorMode = ConveyorMode.stopping;
      } else if (conveyorMode == ConveyorMode.intaking) {
        conveyor.stopMotor();
        conveyorMode = ConveyorMode.stopped;
      }
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