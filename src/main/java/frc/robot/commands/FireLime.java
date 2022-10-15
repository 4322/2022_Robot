package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.FiringSolution.FiringSolutionManager;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// This command must be scheduled to run as non-interruptable!

public class FireLime extends CommandBase {
    
  private Kicker kicker;
  private Conveyor conveyor;
  private Shooter shooter;
  private Hood hood;
  private Driveunbun driveunbun;
  private Limelight limelight;
  private double lastLoggedDistance = 0;
  
  private Timer fireStopTimer = new Timer();
  private double stopDelay = 1;

  public FireLime(Kicker kickerSubsystem, Shooter shooterSubsystem, 
      Hood hoodSubsystem, Driveunbun driveunbun, Limelight limelight) {
    kicker = kickerSubsystem;
    conveyor = Conveyor.getSingleton();
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    this.driveunbun = driveunbun;
    this.limelight = limelight;

    // stop updating firing solution so everything can stabilize
    addRequirements(kicker, conveyor, shooter, hood, Intake.getSingleton());  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fireStopTimer.reset();
    fireStopTimer.start();
  }

  @Override
  public void execute() {

    if (!limelight.getTargetVisible()) {
      if (fireStopTimer.hasElapsed(stopDelay)) {
        conveyor.autoStop();
        kicker.stop();
        shooter.stop();
        hood.stop();
      }
      return;
    }

    fireStopTimer.reset();

    FiringSolution firingSolution = FiringSolutionManager.getSingleton()
        .calcNewSolution(limelight.getDistance());
    hood.setTargetPosition(firingSolution.getHoodPosition());
    
    if (conveyor.isLoaded()) {
      kicker.setSpeed(firingSolution.getKickerSpeed());
      shooter.setSpeed(firingSolution.getFlywheelSpeed());
      
      if (shooter.isAtSpeed() && kicker.isAtSpeed() && hood.isAtTarget() &&
          (Math.abs(limelight.getHorizontalDegToTarget()) <= 
           (driveunbun.isRobotMoving()? DriveConstants.limeRotMovingToleranceDegrees
                                      : DriveConstants.limeRotNotMovingToleranceDegrees))) {
        if (Math.abs(limelight.getDistance() - lastLoggedDistance) > Constants.ShooterConstants.logIntervalDistIn) {
          DataLogManager.log("Fired Shot:\n" +
                             "Time: " + DriverStation.getMatchTime() + "\n" +
                             "Shooter Speed: " + shooter.getSpeed() + "\n" +
                             "Kicker Speed: " + kicker.getSpeed() + "\n" +
                             "Hood Position: " + hood.getPosition() + "\n" +
                             "Distance: " + limelight.getDistance());
          lastLoggedDistance = limelight.getDistance();
        }
        conveyor.shoot();
      } else {
        conveyor.autoStop();
      }
    } else {
      conveyor.autoAdvanceCargo();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.autoStop();
    kicker.stop();
    shooter.stop();
    hood.stop();
  }

  // Run until time has expired or interrupted
  @Override
  public boolean isFinished() {
    return false;  
  }
}
