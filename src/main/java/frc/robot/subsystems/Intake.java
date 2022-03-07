package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase{

    private CANSparkMax intakeMotor;

    public Intake() {
        if (Constants.intakeEnabled) {
            intakeMotor = new CANSparkMax(Constants.IntakeConstants.motorID, MotorType.kBrushless);
            intakeMotor.restoreFactoryDefaults();
            intakeMotor.setInverted(false);
            intakeMotor.setIdleMode(IdleMode.kBrake);   // don't let balls partially fall into the shooter
            intakeMotor.burnFlash();
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
    
}
