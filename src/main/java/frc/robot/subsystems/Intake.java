package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private WPI_TalonFX intakeMotor;

    public Intake() {
        if (Constants.intakeEnabled) {
            intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.intakeTalon_ID);
            intakeMotor.setInverted(true);
        }
      }
    
      public void intake()
      {
        intakeMotor.set(Constants.IntakeConstants.intake_speed);
      }
    
      public void stop()
      {
        intakeMotor.set(0);
      }
      
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
    
}
