// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.cameras.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private static JoystickButton driveBottomRightButton;
  private static JoystickButton driveButtonSeven;
  private static JoystickButton rotateTopLeftButton;
  private static JoystickButton rotateTopRightButton;
  private static JoystickButton rotateBottomLeftButton;

  private ShuffleboardTab tab;
  private SendableChooser<Integer> autoModeChooser = new SendableChooser<Integer>();

  // The robot's subsystems and commands are defined here...
  private final Webcams webcams = new Webcams();
  private final Limelight limelight = new Limelight();
  private final Driveunbun driveunbun = new Driveunbun(webcams, limelight);
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Conveyor conveyor = new Conveyor();

  // Drive Commands
  private final DriveManual driveManual = new DriveManual(driveunbun, limelight);

  // Shooter Commands
  private final StartFiring startFiring = new StartFiring(kicker, conveyor, shooter, hood, 0);
  private final StartFiringOverride startFiringOverride = new StartFiringOverride(conveyor);
  private final StopFiring stopFiring = new StopFiring(kicker, conveyor, shooter, hood);

  // Intake Commands
  private final IntakeIn intakeIn = new IntakeIn(intake, conveyor, 0);
  private final IntakeOut intakeOut = new IntakeOut(intake);

  // Hood Commands
  private final HoodReset hoodReset = new HoodReset(hood);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the 
    // constructors lower CAN bus utilization to make configuration reliable.
    driveunbun.init();
    shooter.init();
    kicker.init();
    intake.init();
    hood.init();
    conveyor.init();

    // Configure the button bindings
    configureButtonBindings();

    tab = Shuffleboard.getTab("Auto");
    autoModeChooser.setDefaultOption("Preload Only", 1);
    autoModeChooser.addOption("Preload + 1", 2);
    autoModeChooser.addOption("Preload + 2", 3);
    autoModeChooser.addOption("Quintet", 5);

    tab.add("Auto Mode", autoModeChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 0)
      .withSize(4, 2);

    if (Constants.driveEnabled) {
      driveunbun.setDefaultCommand(driveManual);
    }
  }

  public void disableSubsystems() {
    driveunbun.setCoastMode();
    hood.setCoastMode();
    conveyor.setCoastMode();
    intake.setCoastMode();
    kicker.setCoastMode();
  }

  public void enableSubsystems() {
    driveunbun.setBrakeMode();
    hood.setBrakeMode();
    conveyor.setBrakeMode();
    intake.setBrakeMode();
    kicker.setBrakeMode();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    coPilot.x.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh));
    coPilot.y.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.outsideTarmac));
    coPilot.b.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac));
    coPilot.a.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderLow));

    coPilot.dPad.up.whileHeld(new CalcFiringSolution(kicker, shooter, hood, limelight));
    coPilot.lb.whenPressed(stopFiring);

    coPilot.rb.whileHeld(intakeIn);
    coPilot.rt.whileHeld(intakeOut);

    coPilot.lt.whileHeld(startFiring);
    coPilot.start.whileHeld(startFiringOverride);

    // to be fixed
    // coPilot.back.whenPressed(hoodReset);

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      driveTopLeftButton = new JoystickButton(driveStick, 5);
      driveBottomLeftButton = new JoystickButton(driveStick, 3);
      driveBottomRightButton = new JoystickButton(driveStick, 4);
      driveTopRightButton = new JoystickButton(driveStick, 6);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      rotateTopLeftButton = new JoystickButton(rotateStick, 5); 
      rotateTopRightButton = new JoystickButton(rotateStick, 6); 
      rotateBottomLeftButton = new JoystickButton(rotateStick, 3); 
      driveTopLeftButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.fieldCentric));
      driveBottomLeftButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.leftCamCentric));
      driveBottomRightButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.rightCamCentric));
      driveTopRightButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.frontCamCentric));
      rotateTopLeftButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.killFieldCentric));
      rotateTopRightButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.sideKillFieldCentric));
      rotateBottomLeftButton.whenPressed(() -> driveunbun.setDriveMode(Driveunbun.DriveMode.limelightFieldCentric));
      driveButtonSeven.whenPressed(new ResetFieldCentric(driveunbun));
    }
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch(autoModeChooser.getSelected()) {
      case 1:
        return new SequentialCommandGroup(
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
          new HoodReset(hood),
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
          new StartFiring(kicker, conveyor, shooter, hood, 2),
          new StopFiring(kicker, conveyor, shooter, hood),
          new DriveRobotCentric(driveunbun, 0, 0.7, 0, 1)
        );
      case 2:
        return new SequentialCommandGroup(
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
          new HoodReset(hood),
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
          new StartFiring(kicker, conveyor, shooter, hood, 2),
          new ParallelCommandGroup(
            new IntakeIn(intake, conveyor, 1.4),
            new DrivePolar(driveunbun, -168, 0.6, 90, 1.2)
          ),
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.outsideTarmac),
          new DrivePolar(driveunbun, 0, 0, 28, 1),
          new StartFiring(kicker, conveyor, shooter, hood, 2),
          new StopFiring(kicker, conveyor, shooter, hood)
        );
      case 3:
        return new SequentialCommandGroup(
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh),
          new HoodReset(hood),
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh),
          new StartFiring(kicker, conveyor, shooter, hood, 2),
          new ParallelCommandGroup(
            new IntakeIn(intake, conveyor, 3.3),
            new SequentialCommandGroup(
              new DrivePolar(driveunbun, 90, 0.6, -69, 0.1),
              new DrivePolar(driveunbun, 92, 0.6, 0, 1.3),
              new DrivePolar(driveunbun, 92, 0.4, 0, 0.2),
              new DrivePolar(driveunbun, -150, 0.6, -60, 1.7)
            )
          ),
          new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.outsideTarmac),
          new DrivePolar(driveunbun, 0, 0, -45, 1),
          new StartFiring(kicker, conveyor, shooter, hood, 4),
          new StopFiring(kicker, conveyor, shooter, hood)
        );
      case 5:
      return new SequentialCommandGroup(
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh),
        new HoodReset(hood),
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh),
        new StartFiring(kicker, conveyor, shooter, hood, 2),
        new ParallelCommandGroup(
          new IntakeIn(intake, conveyor, 3.3),
          new SequentialCommandGroup(
            new DrivePolar(driveunbun, 90, 0.6, -69, 0.1),
            new DrivePolar(driveunbun, 92, 0.6, 0, 1.3),
            new DrivePolar(driveunbun, 92, 0.4, 0, 0.2),
            new DrivePolar(driveunbun, -150, 0.6, -60, 1.7)
          )
        ),
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.outsideTarmac),
        new DrivePolar(driveunbun, 0, 0, -45, 1),
        new StartFiring(kicker, conveyor, shooter, hood, 4),
        new ParallelCommandGroup(
          new IntakeIn(intake, conveyor, 2.7),
          new SequentialCommandGroup(
            new DrivePolar(driveunbun, 140, 0.6, 45, 1.7),
            new DrivePolar(driveunbun, 140, 0.3, 45, 0.2),
            new DrivePolar(driveunbun, -40, 0.3, 45, 0.2),
            new DrivePolar(driveunbun, 0, 0, 45, 0.5)
          )
        ),
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.farLaunchpad),
        new DrivePolar(driveunbun, 0, 0, -30, 1),
        new StartFiring(kicker, conveyor, shooter, hood, 4),
        new StopFiring(kicker, conveyor, shooter, hood)
      );
    }
    return null;
  }

  public void hoodReset() {
    if (!hood.isInitialHomed()) {
      hoodReset.schedule(false); // doesn't allow interrupts
    }
  }

  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
  private static int nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextVerySlowStatusPeriodSparkMs = Constants.verySlowStatusPeriodSparkBaseMs;
  
  public static int nextFastStatusPeriodMs() {
    if (nextFastStatusPeriodMs > Constants.fastStatusPeriodMaxMs) {
      nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
    }
    return nextFastStatusPeriodMs++;
  }

  public static int nextShuffleboardStatusPeriodMs() {
    if (nextShuffleboardStatusPeriodMs > Constants.shuffleboardStatusPeriodMaxMs) {
      nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
    }
    return nextShuffleboardStatusPeriodMs++;
  }

  public static int nextSlowStatusPeriodMs() {
    if (nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs++;
  }

  public static int nextVerySlowStatusPeriodSparkMs() {
    nextVerySlowStatusPeriodSparkMs += 11;
    return nextVerySlowStatusPeriodSparkMs;
  }

  // Stagger status frames from Talon FX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonFX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
      RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
      RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }

  // Stagger status frames from Talon SRX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonSRX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
      RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 
      RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }
}