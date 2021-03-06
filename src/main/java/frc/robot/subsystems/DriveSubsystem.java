/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.*;

import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  /*
   * private WPI_TalonSRX m_leftFront = new WPI_TalonSRX
   * (Constants.cDriveLeftFront); private WPI_TalonSRX m_leftBack = new
   * WPI_TalonSRX (Constants.cDriveLeftBack); private WPI_TalonSRX m_rightFront =
   * new WPI_TalonSRX (Constants.cDriveRightFront); private WPI_TalonSRX
   * m_rightBack = new WPI_TalonSRX (Constants.cDriveRightBack);
   */

  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.cDriveLeftFront);
  private WPI_TalonSRX m_leftBack = new WPI_TalonSRX(Constants.cDriveLeftBack);
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.cDriveRightFront);
  private WPI_TalonSRX m_rightBack = new WPI_TalonSRX(Constants.cDriveRightBack);

  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);
  private PigeonIMU m_imu = new PigeonIMU(Constants.kIMU);

  DifferentialDriveOdometry m_odometry;
  Pose2d m_estimatedPosition;

  private static DriveSubsystem m_instance;

  // private constructor to avoid client applications to use constructor

  // static block initialization for exception handling
  static {
    try {
      m_instance = new DriveSubsystem();
    } catch (Exception e) {
      throw new RuntimeException("Exception occured in creating singleton instance of DriveSubsystem");
    }
  }

  public static DriveSubsystem getInstance() {
    return m_instance;
  }

  /**
   * Creates a new DriveSystem.
   */
  private DriveSubsystem() {
    super();

    //Pose2d currentPose = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(12), new Rotation2d());

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0), Constants.kStartingPosition1);
    m_estimatedPosition = new Pose2d(0.0, 0.0, new Rotation2d());

  }

  public void initControlSystem() {

    System.out.println("[DriveSubsystem.initControlSystem]");

    zeroGyro();
    zeroEncoders();

    m_rightFront.configFactoryDefault();
    m_leftFront.configFactoryDefault();

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_rightFront.setInverted(true);
    m_leftFront.setInverted(false);

    m_rightFront.setSensorPhase(false);
    m_leftFront.setSensorPhase(false);

    m_leftBack.follow(m_leftFront);
    m_leftBack.setInverted(InvertType.FollowMaster);

    m_rightBack.follow(m_rightFront);
    m_rightBack.setInverted(InvertType.FollowMaster);

    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);

    m_rightFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_rightFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_rightFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_rightFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    m_rightFront.config_kF(0, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
    m_rightFront.config_kP(0, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
    m_rightFront.config_kI(0, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
    m_rightFront.config_kD(0, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);

    m_leftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_leftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_leftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_leftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    m_leftFront.config_kF(0, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
    m_leftFront.config_kP(0, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
    m_leftFront.config_kI(0, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
    m_leftFront.config_kD(0, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);

    // m_rightFront.configMotionAcceleration(Constants.kMaxTelopAccelerationInSensorUnits,
    // Constants.kTimeoutMs);
    // m_leftFront.configMotionAcceleration(Constants.kMaxTelopAccelerationInSensorUnits,
    // Constants.kTimeoutMs);

    zeroGyro();
    zeroEncoders();

  }


  public void IMUInit() {
    m_imu.setYaw(0);
  }

  public void zeroGyro() {

    System.out.println("[DriveSubsystem.zeroSensors] initializing sensors");
    m_navXMP.zeroYaw();
    m_navXMP.resetDisplacement();
    m_navXMP.reset();
    IMUInit();
  }

  public void zeroEncoders() {

    m_rightFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
  }

  public void zeroPose() {
    zeroGyro();
    zeroEncoders();
  }

  public void stop() {
    m_leftFront.stopMotor();
    m_rightFront.stopMotor();
  }

  public double wheelDistance(double pulses) {

    double distance = (pulses / Constants.kFullRotationPulses) * Constants.kWheelCircumference;

    return distance;
  }

  public double rightWheelDistance() {
    double pulses = m_rightFront.getSensorCollection().getQuadraturePosition();
    return wheelDistance(pulses);
  }

  public double leftWheelDistance() {
    double pulses = m_leftFront.getSensorCollection().getQuadraturePosition();
    return wheelDistance(pulses);
  }

  public void drive(double left, double right) {

    // speed comes in in meters/sec needs to be ticks/100ms

    double leftVelocity = left * Constants.kEncoderTicksPerMeter * .1;
    double rightVelocity = right * Constants.kEncoderTicksPerMeter * .1;
    m_leftFront.set(ControlMode.Velocity, leftVelocity);
    m_rightFront.set(ControlMode.Velocity, rightVelocity);

    // SmartDashboard.putNumber("left Speed m/s", left);
    // SmartDashboard.putNumber("right Speed m/s", right);
    // SmartDashboard.putNumber("left Speed ticks/100ms", leftVelocity);
    // SmartDashboard.putNumber("right Speed ticks/100ms", rightVelocity);
  }

  public int getLeftError() {
    return m_leftFront.getClosedLoopError();
  }

  public int getRightError() {
    return m_rightFront.getClosedLoopError();
  }

  public int getLeftEncoder() {
    return m_leftFront.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return m_rightFront.getSelectedSensorPosition();
  }

  public int getEncoderSum() {
    return m_rightFront.getSelectedSensorPosition(0);
  }

  public int getEncoderDifference() {
    return m_rightFront.getSelectedSensorPosition(1);
  }

  public double getAngle() {

    return m_navXMP.getYaw();
  }

  public double getPitch() {
    return m_navXMP.getPitch();
  }

  public double getRoll() {
    return m_navXMP.getRoll();
  }

  @Override
  public void periodic() {

    updateOdometry();
    updateDashboard();

  }

  public Pose2d getPositionOnField() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPosition() {
    return m_estimatedPosition;
  }

  public void updateOdometry() {

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    // TODO: Actually we are not negating the angle.. 

    double rad = Units.degreesToRadians(getAngle());
    Rotation2d gyroAngle = new Rotation2d(rad);

    double leftEncoder = m_leftFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;
    double rightEncoder = m_rightFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;

    m_odometry.update(gyroAngle, leftEncoder, rightEncoder);
  }

  public void overwriteOdometry(Pose2d currentPose, Rotation2d rotation) {
    zeroEncoders();

    //TODO: look at the rotation value being sent to resetPosition.  the documentation says this is the GyroAngle - not sure how that is different than the rotation in the Pose2d.

    m_odometry.resetPosition(currentPose, rotation);

    // SmartDashboard.putNumber("Field X Calculated",
    // currentPose.getTranslation().getX());
    // SmartDashboard.putNumber("Field Y Calculated",
    // currentPose.getTranslation().getY());
    // SmartDashboard.putNumber("Field Heading Calculated", rotation.getDegrees());
  }

  public void updateDashboard() {

    double x = getPositionOnField().getTranslation().getX();
    double y = getPositionOnField().getTranslation().getY();
    double angle = getPositionOnField().getRotation().getDegrees();
    SmartDashboard.putNumber("Field X", Units.metersToInches(x));
    SmartDashboard.putNumber("Field Y", Units.metersToInches(y));
    SmartDashboard.putNumber("Field heading", angle);

    float compassHeading = m_navXMP.getCompassHeading();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");
    NetworkTableEntry tableDistanceToTarget = table.getEntry("compass Heading");
    tableDistanceToTarget.setDouble(compassHeading);

    // SmartDashboard.putNumber("Compass", compassHeading);

    // SmartDashboard.putNumber("left distance", leftWheelDistance());
    // SmartDashboard.putNumber("right distance", rightWheelDistance());

    // SmartDashboard.putNumber("left velocity",
    // m_leftFront.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("right velocity",
    // m_rightFront.getSelectedSensorVelocity());

    // SmartDashboard.putNumber("gyro", getAngle());

  }
}
