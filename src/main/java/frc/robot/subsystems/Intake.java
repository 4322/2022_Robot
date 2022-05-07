package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Intake extends SubsystemBase{

  private CANSparkMax intakeMotor;
  private SparkMaxPIDController intakePID;

  private double target;

  private ShuffleboardTab tab;
  
  private NetworkTableEntry power;
  private NetworkTableEntry currentRPM;
  private NetworkTableEntry targetRPM;
  private NetworkTableEntry override;
  private RelativeEncoder intakeEncoder;

  private boolean stalled = false;
  private boolean stallTimerEnabled = false;
  private Timer stallTimer = new Timer();

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
        RobotContainer.nextSlowStatusPeriodMs());
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 
        RobotContainer.nextVerySlowStatusPeriodSparkMs());  
    }
  }

  public void init() {
    if (Constants.intakeEnabled) {
      intakeMotor.restoreFactoryDefaults();
      intakeMotor.setOpenLoopRampRate(IntakeConstants.rampRate);
      intakeMotor.setInverted(true);
      intakeEncoder = intakeMotor.getEncoder();

      intakePID = intakeMotor.getPIDController();
      intakePID.setP(IntakeConstants.kP);
      intakePID.setI(IntakeConstants.kI);
      intakePID.setD(IntakeConstants.kD);
      intakePID.setIZone(IntakeConstants.kIz);
      intakePID.setFF(IntakeConstants.kFF);  

      intakeMotor.burnFlash();
      setCoastMode();  // Allow manual movement until enabled

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Intake");
      
        power =
          tab.add("Power", 0)
          .withPosition(0,0)
          .withSize(1,1)
          .getEntry();

        currentRPM =
          tab.add("Current RPM", 0)
          .withPosition(1,0)
          .withSize(1,1)
          .getEntry();

        targetRPM =
          tab.add("Target RPM", 0)
          .withPosition(2,0)
          .withSize(1,1)
          .getEntry();

        override = 
          tab.add("Override", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(0,1)
          .withSize(1,1)
          .getEntry();
      }
    }
  }

  public void manualIntake() {
    if (Constants.intakeEnabled) {
      resetStalled();
      setSpeed(IntakeConstants.intakeVelocity);
      intakeManualMode = IntakeManualMode.intaking;
    }
  }

  public void manualEject() {
    if (Constants.intakeEnabled) {
      resetStalled();
      setSpeed(IntakeConstants.ejectVelocity);
      intakeManualMode = IntakeManualMode.ejecting;
    }
  }

  public void manualStop() {
    if (Constants.intakeEnabled) {
      if (intakeAutoMode == IntakeAutoMode.stopped) {
        intakeMotor.stopMotor();
      } else { // restart auto intake after eject
        setSpeed(IntakeConstants.intakeVelocity);
      }
      intakeManualMode = IntakeManualMode.stopped;
    }
  }

  // safe to call without requiring the subsystem
  // called periodically
  public void autoIntake() {
    if (Constants.intakeEnabled) {
      if (intakeManualMode == IntakeManualMode.stopped) {
        setSpeed(IntakeConstants.intakeVelocity);
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
    if (Constants.intakeEnabled) {
      if ((Math.abs(intakeEncoder.getVelocity()) < IntakeConstants.minRunVel) && 
        ((intakeAutoMode != IntakeAutoMode.stopped) ||
         (intakeManualMode != IntakeManualMode.stopped))) {
        if (!stallTimerEnabled) {
          stallTimer.start();
          stallTimerEnabled = true;
        }
        if (stallTimer.hasElapsed(IntakeConstants.stallTimeoutSec)) {
          if (!stalled) {
            intakeMotor.stopMotor();
            DriverStation.reportError("Intake Stalled! Attempt to fix in manual mode", false);
            stalled = true;
          }
        }
      // will not reset stall status when intake is stopped
      } else if (!stalled) {
        resetStalled();
      }
      if (Constants.debug) {
        if (override.getBoolean(false) && (target != targetRPM.getDouble(0))) {
          setSpeed(targetRPM.getDouble(0));
        }
        power.setDouble(intakeMotor.getAppliedOutput());
        currentRPM.setDouble(getSpeed());
      }
    }
  }

  private void setSpeed(double rpm) {
    if (Constants.intakeEnabled) {
      if (!stalled) {
        intakePID.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        target = rpm;
      }
      if (Constants.debug) {
        targetRPM.setDouble(rpm);
      }
    }
  }

  private double getSpeed() {
    if (Constants.intakeEnabled) {
      return intakeEncoder.getVelocity();
    } else {
      return -1;
    }
  }

  private void resetStalled() {
    stallTimer.stop();
    stallTimer.reset();
    stallTimerEnabled = false;
    stalled = false;
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
