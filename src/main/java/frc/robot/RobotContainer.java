// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Define controllers
  public static Joystick driveStick = new Joystick(0);
  public static XboxController coPilot = new XboxController(2);

  private static JoystickButton driveTopLeftButton = new JoystickButton(driveStick, 5);
  private static JoystickButton driveBottomLeftButton = new JoystickButton(driveStick, 3);
  private static JoystickButton driveTopRightButton = new JoystickButton(driveStick, 6);

  // The robot's subsystems and commands are defined here...
  private final Driveunbun driveunbun = new Driveunbun();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Conveyor conveyor = new Conveyor();

  // Drive Commands
  private final Drive_Manual driveManual = new Drive_Manual(driveunbun);

  // Shooter Commands
  private final Shooter_Stop stopShooter = new Shooter_Stop(shooter);

  // Intake Commands
  private final Intake_Intake intakeIntake = new Intake_Intake(intake);
  private final Intake_Stop stopIntake = new Intake_Stop(intake);

  // Hood Commands
  private final Hood_Manual hoodManual = new Hood_Manual(hood);
  private final Hood_Reset hoodReset = new Hood_Reset(hood);

  // Conveyor Commands
  private final Conveyor_Stop stopConveyor = new Conveyor_Stop(conveyor);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveunbun.setDefaultCommand(driveManual);
    conveyor.setDefaultCommand(stopConveyor);
  }

  public void disableSubsystems() {
    driveunbun.setCoastMode();
  }

  public void enableSubsystems() {
    driveunbun.setBrakeMode();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveTopLeftButton.whenPressed(new ResetFieldCentric(driveunbun));
    coPilot.b.whenPressed(new Shooter_ManualEject(shooter, conveyor, 3000.0, coPilot));
    coPilot.a.whenPressed(stopShooter);
    coPilot.rt.whenHeld(intakeIntake);
    coPilot.lt.whenHeld(stopIntake);
    coPilot.rb.whenHeld(hoodManual);
    coPilot.lb.whenHeld(hoodReset);
    driveTopLeftButton.whenPressed(new SetToFieldCentric(driveunbun));
    driveBottomLeftButton.whenPressed(new ResetFieldCentric(driveunbun));
    driveTopRightButton.whenPressed(new SetToRobotCentric(driveunbun));
   }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}