package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
    
    /** An example command that uses an example subsystem. */
public class Climb extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
    /**
     * Creates a new Drive_Manual.
     *
     * @param subsystem The subsystem used by this command.
     */

    private final Climber climber;

    public Climb(Climber climbsubsystem) {
        climber = climbsubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    }
}