/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight extends SubsystemBase {
  
  NetworkTable table;
            
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry ledMode;
  NetworkTableEntry camMode;
  NetworkTableEntry pipeline;

  // SHUFFLEBOARD
  ShuffleboardTab tab;
  NetworkTableEntry distanceToTarget;
  NetworkTableEntry targetVisible;

  public Limelight() {
    if (Constants.limelightEnabled) {
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Limelight");

        distanceToTarget = tab.add("Distance to Target", 0)
        .withPosition(1,0)
        .withSize(1,1)
        .getEntry();

        targetVisible = tab.add("Target Visible", false)
        .withPosition(1,0)
        .withSize(1,1)
        .getEntry();
      }
    }
  }

  @Override
  public void periodic() {
    updateLimelightValues();
    if (Constants.debug) {
      distanceToTarget.getDouble(getDistance());
      targetVisible.setBoolean(getTargetVisible());
    }
  }

  public double getX_Offset()
  {
    return tx.getDouble(0);
  }

  public double getY_Offset()
  {
    return ty.getDouble(0);
  }

  public double getTargetArea()
  {
    return ta.getDouble(0);
  }

  public boolean getTargetVisible()
  {
    return tv.getDouble(0.0) == 1.0;
  }

  public void setLed(LedMode mode) {
    ledMode.setNumber(mode.value);
  }

  public void setCamMode(CamMode mode) {
      if (mode == CamMode.VisionProcessor) {
          camMode.setNumber(0);
      } else if (mode == CamMode.DriverCamera) {
          camMode.setNumber(1);
      }
  }

  public enum LedMode {
    Off(1),
    Blink(2),
    On(3);

    private int value;

    LedMode(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }
  }

  public enum CamMode {
    VisionProcessor,
    DriverCamera;
  }

  //Formula Referenced From: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public double getDistance()
  {
    double distance = 0;

    if (getTargetVisible()) {
      double angleToTarget = LimelightConstants.limelightAngle + getY_Offset();
      distance = (LimelightConstants.targetHeight - LimelightConstants.limelightHeight) /
      Math.tan(Math.toRadians(angleToTarget));
    }

    return distance;
  }

  public void updateLimelightValues() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
    pipeline = table.getEntry("pipeline");
  }
}