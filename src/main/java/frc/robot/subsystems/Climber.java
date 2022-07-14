/* Code review comments (delete as each is resolved):
	3.	In Climber.init(), configure the master Talon FX as shown in TalonFXModule.java lines 145-146.
	7.	Create Climber.isAtTarget() following the example in Hood.java. Use a single constant in place of lines 205-206. (Don't know what constant to use)

Start updating ClimbAuto.java to execute a state machine similar to HoodReset.java using the ClimberMode state as follows:
  1. Command the Climber subsystem to set the Talon FX position to 0.
  2. Wait 25ms for the position update command to take effect as shown in HoodReset.java line 56.       Should be done if what I
  3. Move sequentially from positions 1 to 4. Update the ClimberMode states as needed.                  wrote actually works
  4. If the command is interrupted, stop the climber.

No loops are allowed in any of the climber code. All methods must return with no delay. 
We instead use the state machine to keep track of where we left off on the previous iteration.

Determine how you want to abort a climb. We could only continue while the start button is held down, 
but we can't restart the entire sequence if the operator accidentally releases the button because the 
climber would no longer be in initial position. The other buttons aren't needed once we start climbing, 
so we could redefine one of them as the abort. We will figure out how to handle automatic feedback, 
such as a motor stall or robot tilt, with Torsten on Wednesday.
*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.ClimbAuto;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
      climberLeft.configClosedloopRamp(ClimberConstants.rampRate);  
      climberLeft.configOpenloopRamp(ClimberConstants.rampRate);    // for PID tuning
      configCurrentLimit(climberLeft);
      configCurrentLimit(climberRight);
      climberRight.follow(climberLeft);
      climberLeft.setInverted(false);
      climberRight.setInverted(true);
      climberLeft.setNeutralMode(NeutralMode.Brake);
      climberRight.setNeutralMode(NeutralMode.Brake);

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
      talon.configVoltageCompSaturation(ClimberConstants.configVoltageCompSaturation);
      talon.enableVoltageCompensation(ClimberConstants.enableVoltageCompensation);
      talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
			  ClimberConstants.statorEnabled, 
			  ClimberConstants.statorLimit, 
			  ClimberConstants.statorThreshold, 
			  ClimberConstants.statorTime));
		  talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
			  ClimberConstants.supplyEnabled, 
			  ClimberConstants.supplyLimit, 
			  ClimberConstants.supplyThreshold, 
			  ClimberConstants.supplyTime));
    }



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
  
  public void stop() {
    climberLeft.stopMotor();
  }

  private double getPosition() {
    if (Constants.climberEnabled) {
      return climberLeft.getSelectedSensorPosition(0);
    } else {
      return -1;
    }
  }

  public boolean isAtTarget() {
    if (!Constants.climberEnabled) {
      return true;
    }
    return (climberLeft.getClosedLoopError() <=
      (0));
  }

  
  public void moveToPosition(double pos) {
    if (Constants.climberEnabled){
      climberLeft.set(ControlMode.Position, pos);
    }
  }

  public void setCoastMode() {
		climberLeft.setNeutralMode(NeutralMode.Coast);
		climberRight.setNeutralMode(NeutralMode.Coast);
	}

	public void setBrakeMode() {
		climberLeft.setNeutralMode(NeutralMode.Brake);
		climberRight.setNeutralMode(NeutralMode.Brake);
	}
}