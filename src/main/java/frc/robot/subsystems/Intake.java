package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Intake extends SubsystemBase{

  private CANSparkMax intakeMotor;

  public Intake() {
    if (Constants.intakeEnabled) {
      intakeMotor = new CANSparkMax(Constants.IntakeConstants.motorID, MotorType.kBrushless);

      // increase status reporting periods to reduce CAN bus utilization
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 
        RobotContainer.nextSlowStatusPeriodMs());
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());  
    }
  }

  public void init() {
    if (Constants.intakeEnabled) {
      intakeMotor.restoreFactoryDefaults();
      intakeMotor.setOpenLoopRampRate(IntakeConstants.rampRate);
      intakeMotor.setInverted(true);
      intakeMotor.burnFlash();
      setCoastMode();  // Allow manual movement until enabled
    }
  }

  public void intake() {
    if (Constants.intakeEnabled) {
      intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
    }
  }

  public void eject() {
    if (Constants.intakeEnabled) {
      intakeMotor.set(-Constants.IntakeConstants.intakeSpeed);
    }
  }

  public void stop() {
    if (Constants.intakeEnabled) {
      intakeMotor.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setCoastMode() {
    if (Constants.intakeEnabled) {
      intakeMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.intakeEnabled) {
      intakeMotor.setIdleMode(IdleMode.kBrake);
    }
  }
}
