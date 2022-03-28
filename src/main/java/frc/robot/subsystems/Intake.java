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

  private static Intake singleton;

  public enum IntakeManualMode {
    stopped(0),
    intaking(1),
    ejecting(2);

    private int value;

    IntakeManualMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private IntakeManualMode intakeManualMode = IntakeManualMode.stopped;

  public enum IntakeAutoMode {
    stopped(0),
    intaking(1);

    private int value;

    IntakeAutoMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private IntakeAutoMode intakeAutoMode = IntakeAutoMode.stopped;

  private Intake() {
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

  public void manualIntake() {
    if (Constants.intakeEnabled) {
      intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
      intakeManualMode = IntakeManualMode.intaking;
    }
  }

  public void manualEject() {
    if (Constants.intakeEnabled) {
      intakeMotor.set(-Constants.IntakeConstants.intakeSpeed);
      intakeManualMode = IntakeManualMode.ejecting;
    }
  }

  public void manualStop() {
    if (Constants.intakeEnabled) {
      if (intakeAutoMode == IntakeAutoMode.stopped) {
        intakeMotor.stopMotor();
      } else {
        intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
      }
      intakeManualMode = IntakeManualMode.stopped;
    }
  }

  // safe to call without requiring the subsystem
  public void autoIntake() {
    if (Constants.intakeEnabled) {
      if (intakeManualMode == IntakeManualMode.stopped) {
        intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
      }
      intakeAutoMode = IntakeAutoMode.intaking;
    }
  }

  // safe to call without requiring the subsystem
  public void autoStop() {
    if (Constants.intakeEnabled) {
      if (intakeManualMode == IntakeManualMode.stopped) {
        intakeMotor.stopMotor();
      }
      intakeAutoMode = IntakeAutoMode.stopped;
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

  public static Intake getSingleton() {

    if (singleton == null) {
        singleton = new Intake();
    }

    return singleton;
  }
}
