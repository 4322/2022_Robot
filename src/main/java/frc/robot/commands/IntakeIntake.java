package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeIntake extends CommandBase {
    
    private Intake intake;

    public IntakeIntake(Intake intakeSubsytem) {
            // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsytem;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return false;     
  }
}
