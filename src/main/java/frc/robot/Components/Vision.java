/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  // limelight

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // Horizontal offset from crosshair (-29.8, 29.8) degrees
  private NetworkTableEntry m_tx = table.getEntry("tx");
  // Vertical offset from crosshair to target (-24.85, 24.85) degrees
  private NetworkTableEntry m_ty = table.getEntry("ty");
  // Target area in % of image
  private NetworkTableEntry m_ta = table.getEntry("ta");
  // Are there valid targets 0 or 1
  private NetworkTableEntry m_tv = table.getEntry("tv");

  // Are there valid targets 0 or 1
  private NetworkTableEntry m_ts = table.getEntry("ts");

  // Horizontal length of the bounding box
  private NetworkTableEntry m_thor = table.getEntry("thor");

  // Vertical length of the bounding box
  private NetworkTableEntry m_tvert = table.getEntry("tvert");

  private NetworkTableEntry m_ledMode = table.getEntry("ledMode");
  private NetworkTableEntry m_cameraMode = table.getEntry("camMode");

  private CalibrationMap m_calibrationMap;

  /**
   * Creates a new DriveSystem.
   */
  public Vision() {
    super();

    enableCameraMode();
    register();
    calibrate();
  }

  public double tx() {
    return m_tx.getDouble(0.0);
  }

  public double ty() {
    return m_ty.getDouble(0.0);
  }

  public double ta() {
    return m_ta.getDouble(0.0);
  }

  public double ts() {
    return m_ts.getDouble(0.0);
  }

  public double thor() {
    return m_thor.getDouble(0.0);
  }

  public double tvert() {
    return m_tvert.getDouble(0.0);
  }

  public boolean tv() {
    double targets = m_tv.getDouble(0.0);
    return targets == 1.0;
  }

  public void enableCameraMode() {
    m_cameraMode.setNumber(Constants.cLLCameraDriver);
    m_ledMode.setNumber(Constants.cLLLedOff);
  }

  public void enableVisionMode() {
    m_ledMode.setNumber(Constants.cLLLedOn);
    m_cameraMode.setNumber(Constants.cLLCameraVisionProcess);
  }

  // This function dosen't actually do anything right now, it's where we'll document our calibration procedure
  public void calibrate()
  {
    ArrayList<CalibrationPoint> calibrationPoints = new ArrayList<CalibrationPoint>();
    double targetHeight = ((26.0 - 9.0) / 2.0) + 9.0;

    double limelightHeight = 8;
    double centerAboveLimelight = targetHeight - limelightHeight;


    double farCalibrationDistance = 104; // back of the carpet
    double farMeasuredTy = 7.7;

    double calculatedFarTy = Math.atan(centerAboveLimelight / farCalibrationDistance);
    double farOffset = farMeasuredTy - Units.radiansToDegrees(calculatedFarTy);

    CalibrationPoint farPoint = new CalibrationPoint(farCalibrationDistance, farOffset);


    double midCalibrationDistance = 74;
    double midMeasuredTy = 11.11;

    double calculatedMidTy = Math.atan(centerAboveLimelight / midCalibrationDistance);
    double midOffset = midMeasuredTy - Units.radiansToDegrees(calculatedMidTy);

    CalibrationPoint midPoint = new CalibrationPoint(midCalibrationDistance, midOffset);


    double closeCalibrationDistance = 49;
    double closeMeasuredTy = 13.6;

    double calculatedCloseTy = Math.atan(centerAboveLimelight / closeCalibrationDistance);
    double closeOffset = closeMeasuredTy - Units.radiansToDegrees(calculatedCloseTy);

    CalibrationPoint closePoint = new CalibrationPoint(closeCalibrationDistance, closeOffset);

    calibrationPoints.add(closePoint);
    calibrationPoints.add(midPoint);
    calibrationPoints.add(farPoint);
    m_calibrationMap = new CalibrationMap(calibrationPoints);
  }

  public double getDistanceFromTarget()
  {
    // calculate the distance to the target and write it to the network table in
    // inches

    //bottom edge is 9 top is 26


    double targetHeight = ((26.0 - 9.0) / 2.0) + 9.0;

    double limelightHeight = 8;
    double centerAboveLimelight = targetHeight - limelightHeight;
    //double calibrationCrossHairY = 8.75;
    //double limelightAngle = 2.4; //scientifically calculated 7/6/2020 - 3.8
    
    //TODO: This should use our "predicted" distance from target from the odometry
    double limelightAngle = m_calibrationMap.getOffset(0); //Fix this

    double angleAboveLimelight = ty();
    double angleAboveHorizontal = angleAboveLimelight - limelightAngle; //subtraction because limelight is pointed down
    double angleAboveHorizontalInRadians = Units.degreesToRadians(angleAboveHorizontal);
    double tan = Math.tan(angleAboveHorizontalInRadians);
    double distanceToTarget = centerAboveLimelight / tan;


    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

    //double calibratedAngleInRadians = Math.tan(calibrationCrossHairY / calibrationDistanceFromTarget);
    
     // double combinedAngleInRadians = Units.degreesToRadians(ty()) - limelightAngle; //angle above horizontal
     //double distanceToTarget = centerOfTarget / Math.tan(combinedAngleInRadians);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");
    NetworkTableEntry tableDistanceToTarget = table.getEntry("DistanceToTarget");
    tableDistanceToTarget.setDouble(distanceToTarget);
    return distanceToTarget;
  }

  @Override
  public void periodic() {
    getDistanceFromTarget();
  }
}
