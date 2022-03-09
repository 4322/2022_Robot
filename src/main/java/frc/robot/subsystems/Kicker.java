package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.KickerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Kicker extends SubsystemBase {

  private CANSparkMax kicker;

  public Kicker() {
    if (Constants.kickerEnabled) {
      kicker = new CANSparkMax(KickerConstants.kickerID, MotorType.kBrushless);
      kicker.setOpenLoopRampRate(KickerConstants.rampRate);
      kicker.restoreFactoryDefaults();
      kicker.setInverted(false);
      kicker.setIdleMode(IdleMode.kBrake);   // don't let balls partially fall into the shooter
      kicker.burnFlash();

      // increase status reporting periods to reduce CAN bus utilization
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 
        RobotContainer.nextVerySlowStatusPeriodMs());
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 
        RobotContainer.nextVerySlowStatusPeriodMs());
      kicker.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodMs());
    }
  }
  
  public void intake() {
    if (Constants.kickerEnabled) {
      kicker.set(KickerConstants.kickerPower);
    }
  }

  public void stop() {
    if (Constants.kickerEnabled) {
      kicker.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setCoastMode() {
    if (Constants.kickerEnabled) {
      kicker.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.kickerEnabled) {
      kicker.setIdleMode(IdleMode.kBrake);
    }
  }
}