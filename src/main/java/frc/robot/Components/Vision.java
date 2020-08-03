/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.opencv.video.KalmanFilter;
import org.opencv.core.MatOfDouble;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfDouble;
//import org.opencv.video.KalmanFilter;
import org.opencv.video.*;
// import edu.wpi.first.vision.*;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Vision extends SubsystemBase {

  // limelight

  private NetworkTable m_limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // Horizontal offset from crosshair (-29.8, 29.8) degrees
  private NetworkTableEntry m_tx = m_limeLightTable.getEntry("tx");
  // Vertical offset from crosshair to target (-24.85, 24.85) degrees
  private NetworkTableEntry m_ty = m_limeLightTable.getEntry("ty");
  // Target area in % of image
  private NetworkTableEntry m_ta = m_limeLightTable.getEntry("ta");
  // Are there valid targets 0 or 1
  private NetworkTableEntry m_tv = m_limeLightTable.getEntry("tv");

  // Are there valid targets 0 or 1
  private NetworkTableEntry m_ts = m_limeLightTable.getEntry("ts");

  // Horizontal length of the bounding box
  private NetworkTableEntry m_thor = m_limeLightTable.getEntry("thor");

  // Vertical length of the bounding box
  private NetworkTableEntry m_tvert = m_limeLightTable.getEntry("tvert");

  private NetworkTableEntry m_ledMode = m_limeLightTable.getEntry("ledMode");
  private NetworkTableEntry m_cameraMode = m_limeLightTable.getEntry("camMode");

  private NetworkTableEntry m_camTran = m_limeLightTable.getEntry("camtran");

  private NetworkTableEntry m_transX = m_limeLightTable.getEntry("TransX");

  // for access to the OzRam Network Table
  private NetworkTable m_OzRamTable = NetworkTableInstance.getDefault().getTable("OzRam");
  private NetworkTableEntry m_estimatedYLocation = m_OzRamTable.getEntry("Estimated Y Location");

  private CalibrationMap m_calibrationMap;
  private double m_cameraYaw = 0.0;




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


  // This function dosen't actually do anything right now, it's where we'll
  // document our calibration procedure
  public void calibrate() {
    ArrayList<CalibrationPoint> calibrationPoints = new ArrayList<CalibrationPoint>();
    double targetHeight = ((26.0 - 9.0) / 2.0) + 9.0;

    double limelightHeight = 8;
    double centerAboveLimelight = targetHeight - limelightHeight;

    double farCalibrationDistance = 83;
    double farMeasuredTy = 9.73;
    double farAngleOfRobot = 0.4;

    double calculatedFarTy = Math.atan(centerAboveLimelight / farCalibrationDistance);
    double farTyDegrees = Units.radiansToDegrees(calculatedFarTy);
    double calculatedTyDegreesWithGyro = farTyDegrees + farAngleOfRobot;
    double farOffset = farMeasuredTy - calculatedTyDegreesWithGyro;

    double noGyroOffset = farMeasuredTy - farTyDegrees;

    CalibrationPoint farPoint = new CalibrationPoint(farCalibrationDistance, farOffset);
    CalibrationPoint noGyroPoint = new CalibrationPoint(farCalibrationDistance, noGyroOffset);

    calibrationPoints.add(farPoint);
    calibrationPoints.add(noGyroPoint);
    m_calibrationMap = new CalibrationMap(calibrationPoints);
  }

  public double getDistanceFromTarget() {
    // calculate the distance to the target and write it to the network table in
    // inches

    // bottom edge is 9 top is 26

    double centerAboveLimelight = Constants.kTargetHeight - Constants.kLimelightHeight;
    // double calibrationCrossHairY = 8.75;
    // double limelightAngle = 2.4; //scientifically calculated 7/6/2020 - 3.8

    // TODO: This should use our "predicted" distance from target from the odometry
    double limelightAngle = m_calibrationMap.getOffset(0); // Fix this
    double noGyroLimelightAngle = m_calibrationMap.getNoGyroOffset();

    double angleOfRobot = DriveSubsystem.getInstance().getRoll();

    SmartDashboard.putNumber("Pitch", angleOfRobot);

    double angleAboveLimelight = ty() - angleOfRobot;
    double angleAboveLimelightNoGyro = ty();

    double angleAboveHorizontal = angleAboveLimelight - limelightAngle; // subtraction because limelight is pointed down
    double angleAboveHorizontalNoGyro = angleAboveLimelightNoGyro - noGyroLimelightAngle;

    double angleAboveHorizontalInRadians = Units.degreesToRadians(angleAboveHorizontal);
    double angleAboveHorizontalInRadiansNoGyro = Units.degreesToRadians(angleAboveHorizontalNoGyro);

    double tan = Math.tan(angleAboveHorizontalInRadians);
    double tanNoGyro = Math.tan(angleAboveHorizontalInRadiansNoGyro);

    double distanceToTarget = centerAboveLimelight / tan;
    double distanceToTargetNoGyro = centerAboveLimelight / tanNoGyro;

    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

    // double calibratedAngleInRadians = Math.tan(calibrationCrossHairY /
    // calibrationDistanceFromTarget);

    // double combinedAngleInRadians = Units.degreesToRadians(ty()) -
    // limelightAngle; //angle above horizontal
    // double distanceToTarget = centerOfTarget / Math.tan(combinedAngleInRadians);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");
    NetworkTableEntry tableDistanceToTarget = table.getEntry("DistanceToTarget");
    NetworkTableEntry noGyroDistanceToTarget = table.getEntry("DistanceToTargetNoGyro");
    tableDistanceToTarget.setDouble(Units.metersToInches(distanceToTarget));
    noGyroDistanceToTarget.setDouble(Units.metersToInches(distanceToTargetNoGyro));

    return distanceToTarget;
  }

  public void computeYLocationOnFieldFromVision() {

    Translation2d targetPosition = Constants.kPowerPort.getTranslation();
    double limelightDistanceToCenterOfRobot = getDistanceFromTarget() + Constants.kRobotLengthMiddle;
    double angleOffOfTarget = getCameraYaw() - tx();
    double offset = Math.sin(Units.degreesToRadians(angleOffOfTarget)) * limelightDistanceToCenterOfRobot;
    double yDistance = targetPosition.getY() - (offset);
    m_estimatedYLocation.setDouble(Units.metersToInches(yDistance));
  }

  public void locationOnField() {

    // get the distance to the limelight -
    // calculate the position of the limelight on the field
    // then take the rotation of the robot and calculate an additional offset vector

    /*
     * 
     * // The local coordinate system, of the robot, has the origin aligned with the
     * middle of the robot Translation2d limelightToCenterOfRobot = new
     * Translation2d(Constants.kRobotLengthMiddle, 0); // distance from the
     * limelight to the middle Rotation2d robotRotationAroundCenter = new
     * Rotation2d(
     * Units.degreesToRadians(-DriveSubsystem.getInstance().getAngle()));
     * Translation2d rotatedLimeLightPosition =
     * limelightToCenterOfRobot.rotateBy(robotRotationAroundCenter);
     * 
     * 
     * 
     * Translation2d targetPosition = new
     * Translation2d(Constants.kPowerPort.getTranslation().getX(),
     * Constants.kPowerPort.getTranslation().getY());
     */

    Translation2d targetPosition = Constants.kPowerPort.getTranslation();

    double limelightDistanceToCenterOfRobot = Units.inchesToMeters(getDistanceFromTarget())
        + Constants.kRobotLengthMiddle;
    double angleToTarget = tx();

    Pose2d currentEstimatedPosition = DriveSubsystem.getInstance().getPositionOnField();
    Rotation2d currentAngle = currentEstimatedPosition.getRotation(); // we'll assume this is correct

    Translation2d baseVector = new Translation2d(limelightDistanceToCenterOfRobot, 0);
    // Rotation2d rotation = Rotation2d.fromDegrees(-(currentAngle.getDegrees() -
    // angleToTarget + 180));
    Rotation2d rotation = Rotation2d.fromDegrees((currentAngle.getDegrees() - angleToTarget) + 180);
    Translation2d targetToRobot = baseVector.rotateBy(rotation);

    Translation2d robotPosition = targetPosition.plus(targetToRobot);

    SmartDashboard.putNumber("guessed X", Units.metersToInches(robotPosition.getX()));
    SmartDashboard.putNumber("guessed Y", Units.metersToInches(robotPosition.getY()));

    /*
     * //Translation2d targetPosition = new Translation2d(Units.inchesToMeters(129),
     * Units.inchesToMeters(51)); Translation2d targetPosition = new
     * Translation2d(Constants.kPowerPort.getTranslation().getX(),
     * Constants.kPowerPort.getTranslation().getY());
     * 
     * double limelightDistanceToCenterOfRobot =
     * Units.inchesToMeters(getDistanceFromTarget()); double angleToTarget = tx();
     * 
     * Pose2d currentEstimatedPosition =
     * DriveSubsystem.getInstance().getPositionOnField(); Rotation2d currentAngle =
     * currentEstimatedPosition.getRotation(); // we'll assume this is correct
     * 
     * Translation2d baseVector = new
     * Translation2d(limelightDistanceToCenterOfRobot, 0); // Rotation2d rotation =
     * Rotation2d.fromDegrees(-(currentAngle.getDegrees() - angleToTarget + 180));
     * Rotation2d rotation = Rotation2d.fromDegrees(-(currentAngle.getDegrees() -
     * (angleToTarget + 180))); Translation2d targetToRobot =
     * baseVector.rotateBy(rotation);
     * 
     * Translation2d robotPosition = targetPosition.plus(targetToRobot);
     * 
     * SmartDashboard.putNumber("guessed X",
     * Units.metersToInches(robotPosition.getX()));
     * SmartDashboard.putNumber("guessed Y",
     * Units.metersToInches(robotPosition.getY()));
     */
  }

  @Override
  public void periodic() {
    locationOnField();
    computeYLocationOnFieldFromVision();
    getDistanceFromTarget();

    double[] values = new double[6];
    values = m_camTran.getDoubleArray(values);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");

    NetworkTableEntry transX = table.getEntry("TransX");
    transX.setDouble(values[0]);

    NetworkTableEntry transY = table.getEntry("TransY");
    transY.setDouble(values[1]);

    NetworkTableEntry transZ = table.getEntry("TransZ");
    transZ.setDouble(values[2]);

    NetworkTableEntry pitch = table.getEntry("Pitch");
    pitch.setDouble(values[3]);

    NetworkTableEntry yaw = table.getEntry("Yaw");
    yaw.setDouble(values[4]);
    m_cameraYaw = values[4];

    NetworkTableEntry roll = table.getEntry("Roll");
    roll.setDouble(values[5]);


  }

  public double getCameraYaw() {
    return m_cameraYaw;
  }
}
