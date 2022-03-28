package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    
  private WPI_TalonSRX hood;

  //SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");
  private NetworkTableEntry hoodPositionTalon;
  private NetworkTableEntry hoodTarget;
  private NetworkTableEntry hoodPower;
  private NetworkTableEntry isHomeIndicator;
  private NetworkTableEntry override;

  private boolean initialHome = false;
  
  public Hood() {
    if (Constants.hoodEnabled) {
      hood = new WPI_TalonSRX(HoodConstants.motorID);
      RobotContainer.staggerTalonStatusFrames(hood);
    }
  }

  public void init() {
    if (Constants.hoodEnabled) {
      hood.configFactoryDefault();
      hood.setInverted(true);
      hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        HoodConstants.kPIDLoopIdx,
        HoodConstants.kTimeoutMs);
      hood.setSensorPhase(false);
      
      /* Config the peak and nominal outputs */
      hood.configNominalOutputForward(HoodConstants.minForwardPower);
      hood.configNominalOutputReverse(HoodConstants.minReversePower);
      hood.configPeakOutputForward(HoodConstants.maxForwardPower);
      hood.configPeakOutputReverse(HoodConstants.maxReversePower);   
          
      hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
        RobotContainer.nextShuffleboardStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
      hood.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
        RobotContainer.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);   
  
      setCoastMode();  // Allow manual movement until enabled

      /**
      * Grab the 360 degree position of the MagEncoder's absolute
      * position, and intitally set the relative sensor to match.
      */
      hood.configAllowableClosedloopError(HoodConstants.kPIDLoopIdx,
        HoodConstants.hoodTolerancePreset,
        HoodConstants.kTimeoutMs);
      hood.config_kP(HoodConstants.kPIDLoopIdx,
        HoodConstants.kP, HoodConstants.kTimeoutMs);
      hood.config_kI(HoodConstants.kPIDLoopIdx,
        HoodConstants.kI, HoodConstants.kTimeoutMs);
      hood.config_kD(HoodConstants.kPIDLoopIdx,
        HoodConstants.kD, HoodConstants.kTimeoutMs);

      int absolutePosition = hood.getSensorCollection().getPulseWidthPosition();

      /* Mask out overflows, keep bottom 12 bits */
      absolutePosition &= 0xFFF;
      if (HoodConstants.kSensorPhase) { absolutePosition *= -1; }
      if (HoodConstants.kMotorInvert) { absolutePosition *= -1; }
      
      /* Set the quadrature (relative) sensor to match absolute */
      hood.setSelectedSensorPosition(absolutePosition,
        HoodConstants.kPIDLoopIdx,
        HoodConstants.kTimeoutMs);

      // DEBUG
      if (Constants.debug) {
        hoodPositionTalon = tab.add("Hood Position (Talon)", 0)
        .withPosition(0,1)   
        .withSize(1,1)
        .getEntry();

        hoodTarget = tab.add("Hood Target", 0)
        .withPosition(1,0)   
        .withSize(1,1)
        .getEntry();
        
        hoodPower = tab.add("Hood Power", 0)
        .withPosition(1,1)
        .withSize(1,1)
        .getEntry();

        isHomeIndicator = tab.add("Is @ home", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0,0)
        .withSize(1,1)
        .getEntry();

        override = tab.add("Override", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(0,2)
        .withSize(1,1)
        .getEntry();
      }
    }   
  }

  @Override
  public void periodic() {
    if (Constants.hoodEnabled) {
      // SHUFFLEBOARD
      if (Constants.debug) {
        if (initialHome && override.getBoolean(false)) {
          setTargetPosition(hoodTarget.getDouble(0));
        }
        hoodPositionTalon.setDouble(getPosition());
        hoodPower.setDouble(hood.getMotorOutputPercent());
        isHomeIndicator.setBoolean(isAtHome());
      }
    }
  }

  public void setCoastMode() {
    if (Constants.hoodEnabled) {
      hood.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setBrakeMode() {
    if (Constants.hoodEnabled) {
      hood.setNeutralMode(NeutralMode.Brake);
    }
  }

  public void setHoodPower(double power) {
    if (Constants.hoodEnabled) {
      double encValue = getPosition();
      double _power = power;
      
      if (_power > 0) {
        if (encValue >= HoodConstants.hoodMaxPosition) {
          hood.stopMotor();
        } else {
          if (encValue >= HoodConstants.hoodMaxPosition - HoodConstants.hoodDecellerationDistance) {
            _power *= (HoodConstants.hoodMaxPosition - encValue) /
                      HoodConstants.hoodDecellerationDistance;
          }
          // let motor controller apply minimum power bound for easier tuning
          hood.set(Math.min(_power, HoodConstants.maxForwardPower));
        }
      } else if (_power < 0) {
        if (encValue <= HoodConstants.hoodMinPosition) {
          hood.stopMotor();
        } else {
          if (encValue <= HoodConstants.hoodDecellerationDistance) {
            _power *= -encValue / HoodConstants.hoodDecellerationDistance;
          }
          // let motor controller apply minimum power bound for easier tuning
          hood.set(Math.max(_power, HoodConstants.maxReversePower));
        }
      } else {
        hood.stopMotor();
      }
    }
  }

  public double getPosition() {
    return hood.getSelectedSensorPosition(0);
  }

  public void moveHome() {
    if (Constants.hoodEnabled) {
      hood.set(HoodConstants.homingPower);
    }
  }

  public void setTargetPosition(double setpoint, boolean overrideInitialHome) {
    if (Constants.hoodEnabled && Conveyor.getSingleton().canShooterStop()) {
      if (initialHome || overrideInitialHome) {
        hood.set(ControlMode.Position, setpoint);
      }
    }
  }

  public void setTargetPosition(double setpoint) {
    setTargetPosition(setpoint, false);
  }

  // This is only valid following a set position command
  public boolean isAtTarget() {
    return (hood.getClosedLoopError() <= 
      (Driveunbun.getDriveMode() == Driveunbun.DriveMode.limelightFieldCentric?
        HoodConstants.hoodToleranceLime : HoodConstants.hoodTolerancePreset));
  }

  public void clearInitialHome() {
    initialHome = false;
  }

  public void setInitiallyHomed() {
    hood.setSelectedSensorPosition(0);
    initialHome = true;
  }

  public void setCurrentPosition(double pos) {
    hood.setSelectedSensorPosition(pos);
  }

  public boolean isInitialHomed() {
    return initialHome;
  }

  public boolean isAtHome() {
    return hood.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public void stop() {
    hood.stopMotor();
  } 
}
