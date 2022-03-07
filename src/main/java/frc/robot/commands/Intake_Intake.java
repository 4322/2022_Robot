package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class Intake_Intake extends InstantCommand {
    
    private Intake intake;

    public Intake_Intake(Intake intakeSubsytem) {
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
  }
}
