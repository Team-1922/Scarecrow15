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
  DifferentialDriveOdometry m_homeOdometry;

  /**
   * Creates a new DriveSystem.
   */
  public DriveSubsystem() {
    super();

    Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d());
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0), currentPose);
    Pose2d homePose = new Pose2d(0.0, 0.0, new Rotation2d());
    m_homeOdometry = new DifferentialDriveOdometry(new Rotation2d(0.0), homePose);
    

  }

  public void initControlSystem() {

    System.out.println("[DriveSubsystem.initControlSystem]");
    
    zeroSensors();

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

  //  m_rightFront.configMotionAcceleration(Constants.kMaxTelopAccelerationInSensorUnits, Constants.kTimeoutMs);
  //  m_leftFront.configMotionAcceleration(Constants.kMaxTelopAccelerationInSensorUnits, Constants.kTimeoutMs);

    zeroSensors();

  }


  /**
   * sets the robots location on the field to the specified settings. This method
   * should only be called once per match.
   *
   * @param x         the x and y location on the field in meters. the position is
   *                  relative to the left corner of the driver station of the
   *                  alliance we are on.
   * @param gyroAngle the current orientation of the robot on the field. an angle
   *                  of 0 should be looking along the X axis of the field towards
   *                  the opposing alliance. the value should be in degrees as
   *                  returned by the gyro
   * 
   */
  public void setStartingPosition(double x, double y, double gyroAngle) {

    double rad = Units.degreesToRadians(gyroAngle);
    Rotation2d angle = new Rotation2d(rad);
    Pose2d position = new Pose2d(x, y, angle);
    m_odometry.resetPosition(position, angle);

    Rotation2d homeAngle = new Rotation2d(rad);
    
    Pose2d homePosition = new Pose2d(x, y, angle);
    m_homeOdometry.resetPosition(homePosition, homeAngle);
  }

  public void IMUInit() {
    m_imu.setYaw(0);
  }

  public void zeroSensors() {

    System.out.println("[DriveSubsystem.zeroSensors] initializing sensors");
    m_navXMP.zeroYaw();
    m_navXMP.resetDisplacement();
    m_navXMP.reset();
    IMUInit();

    m_rightFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
  }

  public void zeroPose() {
    zeroSensors();
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

    m_leftFront.set(left);
    m_rightFront.set(right);

    SmartDashboard.putNumber("left Speed", left);
    SmartDashboard.putNumber("right Speed", right);

  }

  public int getLeftError() {
    return m_leftFront.getClosedLoopError();
  }

  public int getRightError() {
    return m_rightFront.getClosedLoopError();
  }

  public void driveToTarget(int rightTarget, int targetEncoderDistance) {

    m_rightFront.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
    m_rightFront.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

    if (false) {
      m_rightFront.set(ControlMode.MotionMagic, rightTarget, DemandType.AuxPID, 0);
      m_leftFront.follow(m_rightFront, FollowerType.AuxOutput1);
    } else {
      m_rightFront.set(ControlMode.MotionMagic, rightTarget);
      m_leftFront.follow(m_rightFront);
    }

    System.out.println("[drivesubsystem] - drive to target " + rightTarget + " " + targetEncoderDistance);

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

    


    //return m_navXMP.getAngle();
  }

  @Override
  public void periodic() {

    updateOdometry();
    updateDashboard();

  }

  public Pose2d getPositionOnField() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getHomePosition() {
    return m_homeOdometry.getPoseMeters();
  }

  public void updateOdometry() {

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.

    double rad = Units.degreesToRadians(-getAngle());
    Rotation2d gyroAngle = new Rotation2d(rad);

    
    double leftEncoder = m_leftFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;
    double rightEncoder = m_rightFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;

    m_odometry.update(gyroAngle, leftEncoder, rightEncoder);

  }

  public void updateDashboard() {

    double x = getPositionOnField().getTranslation().getX();
    double y = getPositionOnField().getTranslation().getY();
    double angle = getPositionOnField().getRotation().getDegrees();
    SmartDashboard.putNumber("Field X", x);
    SmartDashboard.putNumber("Field Y", y);
    SmartDashboard.putNumber("Field heading", angle);

    float compassHeading = m_navXMP.getCompassHeading();
    SmartDashboard.putNumber ("Compass", compassHeading);

    


    SmartDashboard.putNumber("left distance", leftWheelDistance());
    SmartDashboard.putNumber("right distance", rightWheelDistance());

    SmartDashboard.putNumber("gyro", getAngle());


  }
}
