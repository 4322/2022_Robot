package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase{
    
    private WPI_TalonFX hood;

    //Hood Control
    private boolean homed = false;

    //SHUFFLEBOARD
    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");
    private NetworkTableEntry hoodPositionTalon;
    private NetworkTableEntry hoodPower;
    private NetworkTableEntry isHomeIndicator =
        tab.add("Is @ home", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0,2)
        .withSize(1,1)
        .getEntry();
    
    public Hood() {
        if (Constants.hoodEnabled) {
            hood = new WPI_TalonFX(Constants.HoodConstants.hoodTalon_ID);

            hood.configFactoryDefault();
            hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                                        Constants.HoodConstants.kPIDLoopIdx,
                                                        Constants.HoodConstants.kTimeoutMs);
            hood.setSensorPhase(Constants.HoodConstants.kSensorPhase);
        
            /* Config the peak and nominal outputs */
            hood.configNominalOutputForward(Constants.HoodConstants.minForwardPower, 
                Constants.HoodConstants.kTimeoutMs);
            hood.configNominalOutputReverse(Constants.HoodConstants.minReversePower, 
                Constants.HoodConstants.kTimeoutMs);
            hood.configPeakOutputForward(Constants.HoodConstants.maxForwardPower, Constants.HoodConstants.kTimeoutMs);
            hood.configPeakOutputReverse(Constants.HoodConstants.maxReversePower, Constants.HoodConstants.kTimeoutMs);   
            
            setCoastMode();  //Allow hood to be moved manually

            // DEBUG
            if (Constants.debug) {
                
                hoodPositionTalon = tab.add("Hood Position (Talon)", 0)
                .withPosition(0,1)   
                .withSize(1,1)
                .getEntry();
                hoodPower = tab.add("Hood Power", 0)
                .withPosition(1,1)
                .withSize(1,1)
                .getEntry();
            }
        }   
    }

  @Override
  public void periodic() {
    hoodPositionTalon.setDouble(getPosition());
    isHomeIndicator.setBoolean(isAtHome());
    hoodPower.setDouble(hood.getMotorOutputPercent());
  }

  public void setCoastMode() {
    hood.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    hood.setNeutralMode(NeutralMode.Brake);
  }

  public double getPosition() {
    return hood.getSelectedSensorPosition(0);
  }

  public void setHoodPower(double power)
  {
    double encValue = getPosition();
    double _power = power;
    
    if (_power > 0) {
      if (encValue >= Constants.HoodConstants.hoodMaxPosition) {
        hood.stopMotor();
      } else {
        if (encValue >= Constants.HoodConstants.hoodMaxPosition - Constants.HoodConstants.hoodDecellerationDistance) {
          _power *= (Constants.HoodConstants.hoodMaxPosition - encValue) /
                    Constants.HoodConstants.hoodDecellerationDistance;
        }
        // let motor controller apply minimum power bound for easier tuning
        hood.set(Math.min(_power, Constants.HoodConstants.maxForwardPower));
      }
    } else if (_power < 0) {
      if (encValue <= Constants.HoodConstants.hoodMinPosition) {
        hood.stopMotor();
      } else {
        if (encValue <= Constants.HoodConstants.hoodDecellerationDistance) {
          _power *= -encValue / Constants.HoodConstants.hoodDecellerationDistance;
        }
        // let motor controller apply minimum power bound for easier tuning
        hood.set(Math.max(_power, Constants.HoodConstants.maxReversePower));
      }
    } else {
      hood.stopMotor();
    }
  }

  public void moveHome() {
    hood.set(Constants.HoodConstants.homingPower);
  }

  public void setTargetPosition(double setpoint) {
    hood.set(ControlMode.Position, setpoint);
  }

  // This is only valid following a set position command
  public boolean isAtTarget() {
    return (hood.getClosedLoopError(Constants.HoodConstants.kPIDLoopIdx) <= 
            Constants.HoodConstants.hoodTolerance);
  }

  public void setAtHome() {
    hood.setSelectedSensorPosition(0);
    homed = true;
  }

  public void setNotAtHome() {
    homed = false;
  }

  public boolean isAtHome() {
    return hood.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public boolean isHomed() {
    return homed;
  }
}
