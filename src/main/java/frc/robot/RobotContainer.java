// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Define controllers
  public static Joystick driveStick;
  public static Joystick rotateStick;
  public static XboxController coPilot = new XboxController(2);

  private static JoystickButton driveTopLeftButton;
  private static JoystickButton driveBottomLeftButton;
  private static JoystickButton driveTopRightButton;

  // The robot's subsystems and commands are defined here...
  private final Driveunbun driveunbun = new Driveunbun();
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Conveyor conveyor = new Conveyor();

  // Drive Commands
  private final DriveManual driveManual = new DriveManual(driveunbun);

  // Shooter Commands
  private final ShooterStop stopShooter = new ShooterStop(shooter);

  // Intake Commands
  private final IntakeIn intakeIn = new IntakeIn(intake, conveyor);
  private final IntakeOut intakeOut = new IntakeOut(intake);

  // Hood Commands
  private final HoodReset hoodReset = new HoodReset(hood);

  // Kicker Commands
  private final KickerEnable kickerEnable = new KickerEnable(kicker, conveyor, shooter);

  // Firing Solutions
  private final FiringSolution test = new FiringSolution(3000, 3000, 0);
  private final FiringSolution fenderLow = new FiringSolution(2000, 3000, 0);
  private final FiringSolution fenderHigh = new FiringSolution(3000, 1000, 0);
  private final FiringSolution tarmacEdge = new FiringSolution(3500, 3000, 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    if (Constants.driveEnabled) {
      driveunbun.setDefaultCommand(driveManual);
    }
  }

  public void disableSubsystems() {
    driveunbun.setCoastMode();
    hood.setCoastMode();
    conveyor.setCoastMode();
    intake.setCoastMode();
  }

  public void enableSubsystems() {
    driveunbun.setBrakeMode();
    hood.setBrakeMode();
    conveyor.setBrakeMode();
    intake.setBrakeMode();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    coPilot.x.whenPressed(new SetSpeedAndAngle(shooter, hood, fenderLow));
    coPilot.y.whenPressed(new SetSpeedAndAngle(shooter, hood, test));
    coPilot.b.whenPressed(new SetSpeedAndAngle(shooter, hood, fenderHigh));
    coPilot.a.whenPressed(stopShooter);

    coPilot.rb.whileHeld(intakeIn);
    coPilot.rt.whileHeld(intakeOut);

    coPilot.lt.whileHeld(kickerEnable);

    coPilot.back.whenHeld(hoodReset);

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      driveTopLeftButton = new JoystickButton(driveStick, 5);
      driveBottomLeftButton = new JoystickButton(driveStick, 3);
      driveTopRightButton = new JoystickButton(driveStick, 6);
      if (Constants.driveTwoJoystick) {
        rotateStick = new Joystick(1);
      }
      driveTopLeftButton.whenPressed(new SetToFieldCentric(driveunbun));
      driveBottomLeftButton.whenPressed(new ResetFieldCentric(driveunbun));
      driveTopRightButton.whenPressed(new SetToRobotCentric(driveunbun));
    }
   }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new SetSpeedAndAngle(shooter, hood, tarmacEdge),
      new KickerEnable(kicker, conveyor, shooter),
      new WaitCommand(5), // wait for balls to shoot
      new DriveRobotCentric(driveunbun, 0, -0.7, 0, 3)
    );
  }

  // space out status frame periods so that status frames don't all come in at once
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextVerySlowStatusPeriodMs = Constants.verySlowStatusPeriodBaseMs;
  
  public static int nextSlowStatusPeriodMs() {
    return nextSlowStatusPeriodMs++;
  }

  public static int nextVerySlowStatusPeriodMs() {
    nextVerySlowStatusPeriodMs += 10;
    return nextVerySlowStatusPeriodMs;
  }
}