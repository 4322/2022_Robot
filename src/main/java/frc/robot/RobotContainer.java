// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.cameras.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  private Timer disableTimer = new Timer();

  // Define controllers
  public static Joystick driveStick;
  public static Joystick rotateStick;
  public static XboxController coPilot = new XboxController(2);

  private static JoystickButton driveTopLeftButton;
  private static JoystickButton driveBottomLeftButton;
  private static JoystickButton driveTopRightButton;
  private static JoystickButton driveBottomRightButton;
  private static JoystickButton driveButtonOne;
  private static JoystickButton driveButtonSeven;
  private static JoystickButton rotateTopLeftButton;
  private static JoystickButton rotateTopRightButton;
  private static JoystickButton rotateBottomLeftButton;
  private static JoystickButton rotateButtonSeven;
  private static JoystickButton rotateButtonNine;
  private static JoystickButton rotateButtonTen;

  private ShuffleboardTab tab;
  private SendableChooser<Integer> autoModeChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> autoSubModeChooser = new SendableChooser<Integer>();

  // The robot's subsystems and commands are defined here...
  private final Webcams webcams = new Webcams();
  private final Limelight limelight = new Limelight();
  private final Drive drive = new Drive(webcams, limelight);
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final Intake intake = Intake.getSingleton();
  private final Hood hood = new Hood();
  private final Conveyor conveyor = Conveyor.getSingleton();

  // Drive Commands
  private final DriveManual driveManual = new DriveManual(drive, limelight);

  // Shooter Commands
  private final FirePreset startFiring = new FirePreset(kicker, conveyor, shooter, hood, 0);
  private final FireStop stopFiring = new FireStop(kicker, conveyor, shooter, hood);

  // Intake Commands
  private final IntakeIn intakeIn = new IntakeIn(intake, conveyor);
  private final IntakeOut intakeOut = new IntakeOut(intake);

  // Hood Commands
  private final HoodReset hoodReset = new HoodReset(hood);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the 
    // constructors lower CAN bus utilization to make configuration reliable.
    drive.init();
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

    autoSubModeChooser.setDefaultOption("None", 0);
    autoSubModeChooser.addOption("Left Only", 1);
    autoSubModeChooser.addOption("Left and Right", 2);

    tab.add("Preload + 1 Disposal Mode", autoSubModeChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 2)
      .withSize(4, 2);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }
  }

  public void disableSubsystems() {
    hood.setCoastMode();
    conveyor.setCoastMode();
    intake.setCoastMode();
    kicker.setCoastMode();
    disableTimer.reset();
    disableTimer.start();
    drive.stop();
  }

  public void enableSubsystems() {
    drive.setDriveMode(Drive.getDriveMode());  // reset limelight LED state
    drive.setBrakeMode();
    hood.setBrakeMode();
    conveyor.setBrakeMode();
    intake.setBrakeMode();
    kicker.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(DriveConstants.disableBreakSec)) {
      drive.setCoastMode();  // robot has stopped, safe to enter coast mode
      disableTimer.stop();
      disableTimer.reset();
    }
    if (Constants.joysticksEnabled) {
      if (Constants.debug) {
        SmartDashboard.putNumber("Drive Stick X", driveStick.getX());
        SmartDashboard.putNumber("Drive Stick Y", driveStick.getY());
        SmartDashboard.putNumber("Drive Stick Z", driveStick.getZ());
      }
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    if (Constants.demo.inDemoMode) {
      coPilot.y.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.disposal));
      if (Constants.demo.shootingSpeed == Constants.demo.ShootingSpeed.SLOW_POP) {
        coPilot.x.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.demoSlowPop));
      }
      if (Constants.demo.shootingSpeed == Constants.demo.ShootingSpeed.FENDER) {
        coPilot.x.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.demoSlowPop));
        coPilot.a.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderLow));
      }
      if (Constants.demo.shootingSpeed == Constants.demo.ShootingSpeed.MEDIUM) {
        coPilot.x.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.demoMediumPop));
        coPilot.a.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderLow));
      }
    } else {
      coPilot.a.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderLow));
      coPilot.x.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.fenderHigh));
      coPilot.y.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.outsideTarmac));
      coPilot.b.whenPressed(new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac));
    }
    
    //coPilot.dPad.up.whileHeld(new CalcFiringSolution(kicker, shooter, hood, limelight));
    coPilot.lb.whenPressed(stopFiring);

    coPilot.rb.whileHeld(intakeIn);
    coPilot.rt.whileHeld(intakeOut);

    coPilot.lt.whileHeld(startFiring);

    coPilot.back.whenPressed(hoodReset, false);  // non-interruptable

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      driveButtonOne  = new JoystickButton(driveStick, 1);
      driveTopLeftButton = new JoystickButton(driveStick, 5);
      driveBottomLeftButton = new JoystickButton(driveStick, 3);
      driveBottomRightButton = new JoystickButton(driveStick, 4);
      driveTopRightButton = new JoystickButton(driveStick, 6);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      rotateTopLeftButton = new JoystickButton(rotateStick, 5); 
      rotateTopRightButton = new JoystickButton(rotateStick, 6); 
      rotateBottomLeftButton = new JoystickButton(rotateStick, 3); 
      rotateButtonSeven = new JoystickButton(rotateStick, 7);
      rotateButtonNine = new JoystickButton(rotateStick, 9);
      rotateButtonTen = new JoystickButton(rotateStick, 10);

      driveTopLeftButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.fieldCentric));
      driveBottomLeftButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.leftCamCentric));
      driveBottomRightButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.rightCamCentric));
      driveTopRightButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.frontCamCentric));
      rotateTopLeftButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.killFieldCentric));
      rotateTopRightButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.sideKillFieldCentric));
      rotateBottomLeftButton.whenPressed(new SetDriveMode(kicker, shooter, hood,
         drive, limelight, Drive.DriveMode.limelightFieldCentric));
      driveButtonOne.whileHeld(intakeIn);
      driveButtonSeven.whenPressed(new ResetFieldCentric(drive, 0, true));
      rotateButtonNine.whenPressed(new ResetFieldCentric(drive, 21, false)); // 1/2 ball
      rotateButtonTen.whenPressed(new ResetFieldCentric(drive, -69, false)); // 3/5 ball
      rotateButtonSeven.whenPressed(drive::resetDisplacement);
    }
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (Constants.demo.inDemoMode) {
      return null;
    }

    final double maxDriveSpeed = 1.0;
    final double tightTurnDriveSpeed = 0.6;
    final double spinUpMediumSec = 0.6;
    final double shootOneCargoSec = 2.0;  // must already be spun-up
    final double shootTwoCargoSec = 4.0;  // must already be spun-up
    final double intakeAfterArrivalSec = 0.1;  // time to pull cargo in securely
    final double intakeAfterArrivalNoTipSec = 0.3;  // delay for no tipping logic to activate
    final double slowApproachSpeed = 0.3;  // intake without slamming into the side rail / terminal
    final double slowApproachSec = 0.2;  // duration of slow approach
    final double rotateToShootStaticSec = 0.8;
    final double rotateToShootLimelightSec = 0.8;  // already pre-rotated
    final double oneCargoDriveBackSec = 1.0;

    final double ballTwoLeftAutoDriveSec = 1.5;
    final double ballTwoLeftAutoDriveDeg = -168;
    //final double ballTwoLeftAutoApproachDeg = -180;
    final double ballTwoLeftAutoShootDeg = 23;

    final double disposalLeft1DriveSec = 1.0;
    final double disposalLeft1DriveDeg = -55.0;

    final double disposalRightDriveSec = 1.0;
    final double disposalRightDriveDeg = 110;
    final double disposalLeft2DriveSec = 1.4;
    final double disposalLeft2DriveDeg = -70.0;

    final double disposalShootDeg = 175.0;

    final double disposalEndDriveSec = 1.2;
    final double disposalEndDriveDeg = 0;
    final double disposalEndRotateDeg = 0;
    final double disposalShootingSec = 2.5;

    final double ballTwoRightAutoDriveSec = 1.2;
    final double ballTwoRightAutoDriveDeg = 60.5;
    final double ballTwoRightAutoApproachDeg = 90;
    final double ballTwoRightSlowApproachSec = 0.07;

    final double ballThreeDriveSec = 1.9;
    final double ballThreeDriveDeg = -151;
    final double ballThreeShootDeg = -31;

    final double ballFourDriveSec = 1.5;
    final double ballFourDriveDeg = 170;
    final double ballFourApproachDeg = 135;
    
    final double ballFiveDriveSec = 0.4;
    final double ballFiveDriveDeg = -45;
    final double ballFiveWaitSec = 1.0;  // time for human player to roll ball into intake
    final double ballFiveShootApproachSec = 1.0;
    final double ballFiveShootDeg = -30;

    // Start of 1 or 2 ball auto
    SequentialCommandGroup leftAuto = 
      new SequentialCommandGroup(
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
        new HoodResetAuto(hood)
            .raceWith(new IntakeIn(intake, conveyor)),
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
        new FirePreset(kicker, conveyor, shooter, hood, spinUpMediumSec + shootOneCargoSec)
            // doesn't hurt in 1 ball mode
            .alongWith(new DrivePreTurnWheels(drive, ballTwoLeftAutoDriveDeg))
      );

    // Start of 3 or 5 ball auto
    SequentialCommandGroup rightAuto = 
      new SequentialCommandGroup(
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
        new HoodResetAuto(hood)
            .raceWith(new IntakeIn(intake, conveyor)),
        new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.insideTarmac),
        new FirePreset(kicker, conveyor, shooter, hood, spinUpMediumSec + shootOneCargoSec)
            .alongWith(new DrivePreTurnWheels(drive, ballTwoRightAutoDriveDeg)),
        new FireStop(kicker, conveyor, shooter, hood),
        new ParallelRaceGroup(
          new IntakeIn(intake, conveyor),
          new SequentialCommandGroup(
            new DrivePolar(drive, ballTwoRightAutoDriveDeg, tightTurnDriveSpeed, 
                           ballTwoRightAutoApproachDeg - 90, ballTwoRightAutoDriveSec),
            new DrivePolar(drive, ballTwoRightAutoApproachDeg, slowApproachSpeed, 
                           ballTwoRightAutoApproachDeg - 90, ballTwoRightSlowApproachSec),
            new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.cargoRing),
            new DrivePreTurnWheels(drive, ballThreeDriveDeg),
            new DrivePolar(drive, ballThreeDriveDeg, tightTurnDriveSpeed, 
                           ballThreeDriveDeg + 90, ballThreeDriveSec),
            new WaitCommand(intakeAfterArrivalNoTipSec)
          )
        )
      );

    // Reset pose after opposing alliance cargo disposal
    SequentialCommandGroup endDisposal = 
      new SequentialCommandGroup(
        new FirePreset(kicker, conveyor, shooter, hood, disposalShootingSec),
        new FireStop(kicker, conveyor, shooter, hood),
        new DrivePolar(drive, disposalEndDriveDeg, 0, 
                       disposalEndRotateDeg, disposalEndDriveSec)
      );

    switch(autoModeChooser.getSelected()) {

      case 1:
        leftAuto.addCommands(
          new FireStop(kicker, conveyor, shooter, hood),
          new DriveRobotCentric(drive, 0, 0.7, 0, oneCargoDriveBackSec)
        );
        return leftAuto;

      case 2:
        leftAuto.addCommands(
          new ParallelRaceGroup(
            new IntakeIn(intake, conveyor),
            new SequentialCommandGroup(
              new DrivePolar(drive, ballTwoLeftAutoDriveDeg, tightTurnDriveSpeed, 
                            ballTwoLeftAutoDriveDeg + 90, ballTwoLeftAutoDriveSec),
              //new DrivePolar(drive, ballTwoLeftAutoApproachDeg, slowApproachSpeed, 
              //              ballTwoLeftAutoApproachDeg - 90, slowApproachSec),
              new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.cargoRing),
              new WaitCommand(intakeAfterArrivalSec)
            )
          )
        );
        switch (autoSubModeChooser.getSelected()) {
          case 0:
            leftAuto.addCommands(
              new InstantCommand(limelight::enableLed),
              new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.cargoRing),
              new DrivePolar(drive, 0, 0, ballTwoLeftAutoShootDeg, rotateToShootStaticSec)
                  .raceWith(new IntakeIn(intake, conveyor)),
              new RotToTarget(drive, limelight, rotateToShootLimelightSec),
              new CalcFiringSolution(kicker, shooter, hood, limelight),
              new InstantCommand(limelight::disableLed),
              new FirePreset(kicker, conveyor, shooter, hood, shootOneCargoSec),
              new FireStop(kicker, conveyor, shooter, hood)
            );
            break;
          case 1:
            leftAuto.addCommands(
              new InstantCommand(limelight::enableLed),
              new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.cargoRing),
              new DrivePolar(drive, 0, 0, ballTwoLeftAutoShootDeg, rotateToShootStaticSec)
                  .raceWith(new IntakeIn(intake, conveyor)),
              new RotToTarget(drive, limelight, rotateToShootLimelightSec),
              new CalcFiringSolution(kicker, shooter, hood, limelight),
              new InstantCommand(limelight::disableLed),
              new FirePreset(kicker, conveyor, shooter, hood, shootOneCargoSec),
              new FireStop(kicker, conveyor, shooter, hood),
              new DrivePreTurnWheels(drive, disposalLeft1DriveDeg),
              new ParallelRaceGroup(
                new IntakeIn(intake, conveyor),
                new SequentialCommandGroup(
                  new DrivePolar(drive, disposalLeft1DriveDeg, maxDriveSpeed, 
                                 disposalLeft1DriveDeg + 90, disposalLeft1DriveSec),
                  new WaitCommand(intakeAfterArrivalSec),
                  new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.disposal),
                  new DrivePolar(drive, 0, 0, disposalShootDeg, rotateToShootStaticSec)
                )
              ),
              endDisposal
            );
            break;
          case 2:
            leftAuto.addCommands(
              new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.cargoRing),
              new DrivePolar(drive, 0, 0, ballTwoLeftAutoShootDeg, rotateToShootStaticSec)
                  .raceWith(new IntakeIn(intake, conveyor)),
              new FirePreset(kicker, conveyor, shooter, hood, shootOneCargoSec),
              new FireStop(kicker, conveyor, shooter, hood),
              new DrivePreTurnWheels(drive, disposalRightDriveDeg),
              new ParallelRaceGroup(
                new IntakeIn(intake, conveyor),
                new SequentialCommandGroup(
                  new DrivePolar(drive, disposalRightDriveDeg, maxDriveSpeed, 
                                disposalRightDriveDeg - 90, disposalRightDriveSec),
                  new WaitCommand(intakeAfterArrivalSec),
                  new DrivePreTurnWheels(drive, disposalLeft2DriveDeg),
                  new DrivePolar(drive, disposalLeft2DriveDeg, maxDriveSpeed, 
                                disposalLeft2DriveDeg - 90, disposalLeft2DriveSec),
                  new WaitCommand(intakeAfterArrivalSec),
                  new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.disposal),
                  new DrivePolar(drive, 0, 0, disposalShootDeg, rotateToShootStaticSec)
                )
              ),
              endDisposal
            );
            break;
        }
        return leftAuto;

      case 3:
        rightAuto.addCommands(
          new InstantCommand(limelight::enableLed),
          new DrivePolar(drive, 0, 0, ballThreeShootDeg, rotateToShootStaticSec)
              .raceWith(new IntakeIn(intake, conveyor)),
          new RotToTarget(drive, limelight, rotateToShootLimelightSec),
          new CalcFiringSolution(kicker, shooter, hood, limelight),
          new InstantCommand(limelight::disableLed),
          new FirePreset(kicker, conveyor, shooter, hood, shootTwoCargoSec),
          new FireStop(kicker, conveyor, shooter, hood)
        );
        return rightAuto;

      case 5:
        rightAuto.addCommands(
          new DrivePolar(drive, 0, 0, ballThreeShootDeg, rotateToShootStaticSec)
              .raceWith(new IntakeIn(intake, conveyor)),
          new FirePreset(kicker, conveyor, shooter, hood, shootTwoCargoSec),
          new FireStop(kicker, conveyor, shooter, hood),
          new ParallelRaceGroup(
            new IntakeIn(intake, conveyor),
            new SequentialCommandGroup(
              new DrivePolar(drive, ballFourDriveDeg, maxDriveSpeed, 
                             ballFourApproachDeg - 90, ballFourDriveSec),
              new DrivePolar(drive, ballFourApproachDeg, slowApproachSpeed, 
                             ballFourApproachDeg - 90, slowApproachSec),
              new DrivePolar(drive, ballFiveDriveDeg, maxDriveSpeed, ballFiveDriveDeg + 90, ballFiveDriveSec),
              new SetFiringSolution(kicker, shooter, hood, Constants.FiringSolutions.autoBall5),
              new DrivePolar(drive, 0, 0, ballFiveDriveDeg + 90, ballFiveWaitSec)
            )
          ),
          new DrivePolar(drive, ballFiveShootDeg, maxDriveSpeed, 
                         ballFiveShootDeg, ballFiveShootApproachSec),
          new InstantCommand(limelight::enableLed),
          new RotToTarget(drive, limelight, rotateToShootLimelightSec),
          new CalcFiringSolution(kicker, shooter, hood, limelight),
          new InstantCommand(limelight::disableLed),
          new FirePreset(kicker, conveyor, shooter, hood, shootTwoCargoSec),
          new FireStop(kicker, conveyor, shooter, hood)
        );
        return rightAuto;
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