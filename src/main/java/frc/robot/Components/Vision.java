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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
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


  private NetworkTableEntry m_corners = m_limeLightTable.getEntry("tcornxy");

  // for access to the OzRam Network Table
  private NetworkTable m_OzRamTable = NetworkTableInstance.getDefault().getTable("OzRam");
  private NetworkTableEntry m_guessedYLocation = m_OzRamTable.getEntry("Guessed Y");
  private NetworkTableEntry m_guessedXLocation = m_OzRamTable.getEntry("Guessed X");

  private NetworkTableEntry m_slopeBasedGuessedYLocation = m_OzRamTable.getEntry("Slope Guessed Y");
  private NetworkTableEntry m_slopeBasedGuessedXLocation = m_OzRamTable.getEntry("Slope Guessed X");

  private NetworkTableEntry m_cameraGuessedYLocation = m_OzRamTable.getEntry("Camera Guessed Y");
  private NetworkTableEntry m_cameraGuessedXLocation = m_OzRamTable.getEntry("Camera Guessed X");



  private NetworkTableEntry m_ulx = m_OzRamTable.getEntry("ULX");
  private NetworkTableEntry m_uly = m_OzRamTable.getEntry("ULY");
  private NetworkTableEntry m_urx = m_OzRamTable.getEntry("URX");
  private NetworkTableEntry m_ury = m_OzRamTable.getEntry("URY");

  private NetworkTableEntry m_llx = m_OzRamTable.getEntry("LLX");
  private NetworkTableEntry m_lly = m_OzRamTable.getEntry("LLY");
  private NetworkTableEntry m_lrx = m_OzRamTable.getEntry("LRX");
  private NetworkTableEntry m_lry = m_OzRamTable.getEntry("LRY");

  private NetworkTableEntry m_confidence = m_OzRamTable.getEntry("confidence");

  private NetworkTableEntry m_slope = m_OzRamTable.getEntry("slope");

  private NetworkTableEntry m_angleToRightPt = m_OzRamTable.getEntry("angleToRightPt");
  private NetworkTableEntry m_angleToLeftPt = m_OzRamTable.getEntry("angleToLeftPt");
  private NetworkTableEntry m_deadBand = m_OzRamTable.getEntry("Deadband");
  private NetworkTableEntry m_tableDistanceToTarget = m_OzRamTable.getEntry("DistanceToTarget");
  private NetworkTableEntry m_noGyroDistanceToTarget = m_OzRamTable.getEntry("DistanceToTargetNoGyro");
  private NetworkTableEntry m_targetYaw = m_OzRamTable.getEntry("TargetYaw");
  

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
    double farMeasuredTy = 10.23;//9.73; // 10.179; //   was 9.73;
    double farAngleOfRobot = -0.35; //read this directly off the Pitch value from the gyro. Include the sign

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


    m_tableDistanceToTarget.setDouble(Units.metersToInches(distanceToTarget));
    m_noGyroDistanceToTarget.setDouble(Units.metersToInches(distanceToTargetNoGyro));

    return distanceToTarget;
  }

  public Pose2d usingCameraTargetYawGetLocationOnField ()
  {
    Pose2d p = locationOnField(getCameraYaw());
    // record p
    m_cameraGuessedXLocation.setDouble(Units.metersToInches(p.getTranslation().getX()));
    m_cameraGuessedYLocation.setDouble(Units.metersToInches(p.getTranslation().getY()));

    return p;
  }

public Pose2d usingSlopeYawToGetLocationOnField ()
  {
    double slopeYaw = getSlopeYaw();
    Pose2d p = locationOnField(slopeYaw);
    // record p
    m_slopeBasedGuessedXLocation.setDouble(Units.metersToInches(p.getTranslation().getX()));
    m_slopeBasedGuessedYLocation.setDouble(Units.metersToInches(p.getTranslation().getY()));
    
    return p;
  }
  
  public Pose2d locationOnField() {
    if(m_confidence.getDouble(0.0) == 1.0)
    {
      return locationOnField(getTargetYaw()); 
    }
    return null;

  }
  
  private Pose2d locationOnField(double targetYaw) {

    // get the distance to the limelight -
    // calculate the position of the limelight on the field
    // then take the rotation of the robot and calculate an additional offset vector

    Translation2d targetPosition = Constants.kPowerPort.getTranslation();

    double limelightDistanceToCenterOfRobot = getDistanceFromTarget() + Constants.kRobotLengthMiddle; // All in meters
    double angleToTarget = tx();

    Translation2d baseVector = new Translation2d(limelightDistanceToCenterOfRobot, 0);

    double rotationAngle = targetYaw - angleToTarget;

    //mws double currentAngle = getCameraYaw(); // This is our heading
    // Rotation2d rotation = Rotation2d.fromDegrees(-(currentAngle.getDegrees() -
    // angleToTarget + 180));

    // mwsRotation2d rotation = Rotation2d.fromDegrees((currentAngle - angleToTarget) + 180);
    
    Rotation2d rotation = Rotation2d.fromDegrees((rotationAngle) + 180);
    Translation2d targetToRobot = baseVector.rotateBy(rotation);

    Translation2d robotPosition = targetPosition.plus(targetToRobot);
  

    SmartDashboard.putNumber("guessed X", Units.metersToInches(robotPosition.getX()));
    SmartDashboard.putNumber("guessed Y", Units.metersToInches(robotPosition.getY()));
    m_guessedXLocation.setDouble(Units.metersToInches(robotPosition.getX()));
    m_guessedYLocation.setDouble(Units.metersToInches(robotPosition.getY()));

    Pose2d pose = new Pose2d(robotPosition, Rotation2d.fromDegrees(rotationAngle));
    return pose;

  }

  @Override
  public void periodic() {
    locationOnField();

    usingSlopeYawToGetLocationOnField();
    usingCameraTargetYawGetLocationOnField();

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

    double[] corners = new double[20];
    corners = m_corners.getDoubleArray(corners);

    double confidence = confidenceInData(corners);
    m_confidence.setDouble(confidence);

    
    inDeadBand();
    calculateSlope();

    if (confidence == 1.0) {
      orderPoints(corners);
    }
  }

  /**
   * looks at the set of points supplied from the camera and decides how confident
   * we are that the data is good and can be used to make a decision off of
   */
  public double confidenceInData(double[] pts) {

    double confidence = 1;

    if (pts.length != 8) {
      confidence /= 2;
    }

    // check to see that the points adds up to a reasonable size - indicating we are
    // not to far away
    // or that the shape is partially off the screen.

    double minx = 10000;
    double maxx = -10000;
    for (int i = 0; i < pts.length; i += 2) {
      if (pts[i] < minx) {
        minx = pts[i];
      } else if (pts[i] > maxx) {
        maxx = pts[i];
      }
    }

    if ((maxx - minx) < 80) {
      confidence /= 2;
    }

    // add additional tests that looks for problems in the data and factors that
    // into the confidence

    return confidence;
  }

/** looks at several pieces of data and determins if the robot is oriented in a location where it is hard to use the
 * yaw from the Limelight to determine the robots angle to the wall.
 */
  public boolean inDeadBand ()
  {

    boolean deadband = false;

    double ulx = m_ulx.getDouble(0);
    double uly = m_uly.getDouble(0);
    double urx = m_urx.getDouble(1);
    double ury = m_ury.getDouble(0);

    double slope = (uly - ury) / (ulx - urx);
 
    double tx = m_tx.getDouble(0.0);

    // if the robot is fairly square onto the target and 
    // there is little rotation to get it aligned with the center, meaning it is reasonably straight onto the target
    if (Math.abs(slope) < 0.03 && Math.abs(tx) < 10.0) {
      deadband = true;
    }

    m_deadBand.setBoolean(deadband);

    return deadband;
  }

public double calculateSlope()
{

  double ulx = m_ulx.getDouble(0);
  double uly = m_uly.getDouble(0);
  double urx = m_urx.getDouble(1);
  double ury = m_ury.getDouble(0);

    // calculate the slope between the two points that are the "highest" in the view
    double slope = (ury - uly) / (urx - ulx);
    m_slope.setDouble(slope);
 
    return slope;
}

  public double[] orderPoints(double[] pts) {

    double[] results = new double[8];

    if (pts.length < 8) {
      return results;
    }

    double epsilon = 3.0;

    double ulx = 10000;
    double uly = 10000;
    double urx = -10000;
    double ury = 10000;

    double llx = 10000;
    double lly = -10000;
    double lrx = -10000;
    double lry = -10000;

    // find the upper left
    ulx = pts[0];
    uly = pts[1];

    int bestIndex = 0;
    for (int i = 2; i < pts.length; i += 2) {
      if (Math.abs(pts[i] - ulx) < epsilon) { // if it has the same X value as the already identified item
        if (pts[i + 1] < uly) { // see if the y value makes it a better item
          ulx = pts[i];
          uly = pts[i + 1];
          bestIndex = i;
        }
      } else if (pts[i] < ulx) {
        ulx = pts[i];
        uly = pts[i + 1];
        bestIndex = i;
      }
    }
    pts[bestIndex] = -1;
    pts[bestIndex + 1] = -1;

    // now look for the upper right corner

    bestIndex = -1;
    for (int j = 0; j < pts.length; j += 2) {
      if (pts[j] == -1) {
        continue;
      }

      if (Math.abs(pts[j] - urx) < epsilon) { // if it has the same X value as the already identified item
        if (pts[j + 1] < ury) { // see if the y value makes it a better item
          urx = pts[j];
          ury = pts[j + 1];
          bestIndex = j;
        }
      } else if (pts[j] > urx) {
        urx = pts[j];
        ury = pts[j + 1];
        bestIndex = j;
      }
    }
    pts[bestIndex] = -1;
    pts[bestIndex + 1] = -1;

    // now look for the lower right corner
    bestIndex = -1;
    for (int k = 0; k < pts.length; k += 2) {
      if (pts[k] == -1) {
        continue;
      }

      if (Math.abs(pts[k] - lrx) < epsilon) { // if it has the same X value as the already identified item
        if (pts[k + 1] > lry) { // see if the y value makes it a better item
          lrx = pts[k];
          lry = pts[k + 1];
          bestIndex = k;
        }
      } else if (pts[k] > lrx) {
        lrx = pts[k];
        lry = pts[k + 1];
        bestIndex = k;
      }
    }
    pts[bestIndex] = -1;
    pts[bestIndex + 1] = -1;

    bestIndex = -1;
    for (int l = 0; l < pts.length; l += 2) {
      if (pts[l] == -1) {
        continue;
      }

      if (Math.abs(pts[l] - llx) < epsilon) { // if it has the same X value as the already identified item
        if (pts[l + 1] > lly) { // see if the y value makes it a better item
          llx = pts[l];
          lly = pts[l + 1];
          bestIndex = l;
        }
      } else if (pts[l] < llx) {
        llx = pts[l];
        lly = pts[l + 1];
        bestIndex = l;
      }
    }
    pts[bestIndex] = -1;
    pts[bestIndex + 1] = -1;



    results[0] = ulx;
    results[1] = uly;
    results[2] = urx;
    results[3] = ury;
    results[4] = llx;
    results[5] = lly;
    results[6] = lrx;
    results[7] = lry;

    m_ulx.setDouble(results[0]);
    m_uly.setDouble(results[1]);
    m_urx.setDouble(results[2]);
    m_ury.setDouble(results[3]);
    m_llx.setDouble(results[4]);
    m_lly.setDouble(results[5]);
    m_lrx.setDouble(results[6]);
    m_lry.setDouble(results[7]);


    double rightPtYaw = computeRotationAngleToPt(urx);
    double angleToRightPt = Units.radiansToDegrees(rightPtYaw);
    m_angleToRightPt.setDouble(angleToRightPt);

    double leftPtYaw = computeRotationAngleToPt(ulx);
    double angleToLeftPt = Units.radiansToDegrees(leftPtYaw);
    m_angleToLeftPt.setDouble(angleToLeftPt);

    return results;

  }


/** supply a point in pixel coordinates and it will return the angle (yaw) to point the camera directly at that point
 * the return value is in radians 
*/
  public double computeRotationAngleToPt (double ptx) {

    double horizontal_fov = Units.degreesToRadians(54.0);

    double pixelsWide = 320.0 * 3.0;
  

    double x1 = ptx - pixelsWide/2.0; // shift to have the origin in the middle of the view, not the upper left hand corner
    x1 /= pixelsWide;  // scale to display coordinates
    double ax = Math.atan2(x1, 1);

    return ax;
  }


  public double scaleXToDisplayCoords(double x) {

    double horizontal_fov = Units.degreesToRadians(54.0);

    double pixelsWide = 320.0 * 3.0;
    double halfPixelsWide = pixelsWide / 2.0;
    double nx = (1.0/halfPixelsWide) * ((halfPixelsWide - 0.5) - x);

    double vpw = 2.0* Math.tan(horizontal_fov/2.0);

    double dx = vpw/2.0 * nx;
    
    return dx;

  }





  public double scaleYToDisplayCoords (double y) {

    double vertical_fov = Units.degreesToRadians(41.0);

    double pixelsHigh = 240.0 * 3.0;
    double halfPixelsHeight = pixelsHigh / 2.0;
    double ny = (1.0/halfPixelsHeight) * ((halfPixelsHeight - 0.5) - y);

    double vph = 2.0* Math.tan(vertical_fov/2.0);

    double dy = vph/2.0 * ny;
    
    return dy;

  }

  public double computeHeightAngleToPt (double pty) {

    double y = scaleYToDisplayCoords(pty);
    double ay = Math.atan2(y, 1.0);

    return ay;
  }

  public double getSlopeYaw()
  {

    double targetYaw = 0.0;
      //look at the slope and the distance to try and interprolate what the angle should be
      //double slopePerDegree = 0.035 / 10.0;
      double slopePerDegree = (0.069 + 0.046) / 30.0;
      
      double changePerInch = (0.047 - 0.035) / (67.0 - 103.0);
      double distanceToTarget = m_tableDistanceToTarget.getDouble(60.0);

      double distanceFactor = (distanceToTarget - 103.0) * changePerInch;
      double slopeFactor = m_slope.getDouble(0.0) / slopePerDegree;
      targetYaw = -(slopeFactor + distanceFactor);

      return targetYaw;
  }

  public double getTargetYaw() {
    
    double targetYaw = 0.0;
    
    if(!inDeadBand()) {
      targetYaw = getCameraYaw();
    }
    else {
      targetYaw = getSlopeYaw();
      
    }
    
    m_targetYaw.setDouble(targetYaw);

    return targetYaw;

  }

  public double getCameraYaw() {

    return m_cameraYaw;
  }



}
