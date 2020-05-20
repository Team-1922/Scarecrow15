/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import java.lang.Math;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;

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

  DifferentialDriveOdometry m_odometry;

  boolean m_ramsete = false;

  /**
   * Creates a new DriveSystem.
   */
  public DriveSubsystem() {
    super();

    Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d());
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0), currentPose);

  }

  public void initForTankDrive() {

    m_ramsete = false;

    m_leftFront.configFactoryDefault();
    m_rightFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();
    m_leftBack.configFactoryDefault();

    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);

    m_rightBack.set(ControlMode.Follower, m_rightFront.getDeviceID());
    m_rightBack.setInverted(InvertType.FollowMaster);
    m_leftBack.set(ControlMode.Follower, m_leftFront.getDeviceID());
    m_leftBack.setInverted(InvertType.FollowMaster);

    /* Configure neutral deadband */
    m_rightFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    m_leftFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
  }

  public void initForRamseteDrive() {
    m_rightFront.configFactoryDefault();
    m_leftFront.configFactoryDefault();

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_rightFront.setInverted(true);
    m_leftFront.setInverted(false);

    m_rightFront.setSensorPhase(true);
    m_leftFront.setSensorPhase(true);

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

    // public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20,
    // 1023.0/7200.0, 300, 1.00);
    double kI = 1023.0 / 7200.0;

    /* Config the Velocity closed loop gains in slot0 */
    m_rightFront.config_kF(0, 0, Constants.kTimeoutMs);
    m_rightFront.config_kP(0, 0, Constants.kTimeoutMs);
    m_rightFront.config_kI(0, kI, Constants.kTimeoutMs);
    m_rightFront.config_kD(0, 0, Constants.kTimeoutMs);

    m_leftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_leftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_leftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_leftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    m_leftFront.config_kF(0, 0, Constants.kTimeoutMs);
    m_leftFront.config_kP(0, 0, Constants.kTimeoutMs);
    m_leftFront.config_kI(0, kI, Constants.kTimeoutMs);
    m_leftFront.config_kD(0, 0, Constants.kTimeoutMs);

    Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d());
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0), currentPose);

    zeroSensors();

    m_ramsete = true;

  }

  public void initForTalonController() {

    m_ramsete = false;

    m_leftFront.configFactoryDefault();
    m_rightFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();
    m_leftBack.configFactoryDefault();

    m_leftFront.set(ControlMode.PercentOutput, 0);
    m_rightFront.set(ControlMode.PercentOutput, 0);
    m_leftBack.set(ControlMode.PercentOutput, 0);
    m_rightBack.set(ControlMode.PercentOutput, 0);

    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);

    m_rightBack.set(ControlMode.Follower, m_rightFront.getDeviceID());
    m_rightBack.setInverted(InvertType.FollowMaster);
    m_leftBack.set(ControlMode.Follower, m_leftFront.getDeviceID());
    m_leftBack.setInverted(InvertType.FollowMaster);

    /* Configure the left Talon's selected sensor as local QuadEncoder */
    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
        Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout

    /*
     * Configure the Remote Talon's selected sensor as a remote sensor for the right
     * Talon
     */
    m_rightFront.configRemoteFeedbackFilter(m_leftFront.getDeviceID(), // Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
        Constants.REMOTE_0, // Source number [0, 1]
        Constants.kTimeoutMs); // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    m_rightFront.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback
                                                                                                        // Device of
                                                                                                        // Remote Talon
    m_rightFront.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs); // Quadrature
                                                                                                                   // Encoder
                                                                                                                   // of
                                                                                                                   // current
                                                                                                                   // Talon

    /* Setup Difference signal to be used for Turn */
    m_rightFront.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
    m_rightFront.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    m_rightFront.configSelectedFeedbackCoefficient(0.5, // Coefficient
        Constants.PID_PRIMARY, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    /*
     * Configure Difference [Difference between both QuadEncoders] to be used for
     * Auxiliary PID Index
     */
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, Constants.PID_TURN,
        Constants.kTimeoutMs);

    /* Scale the Feedback Sensor using a coefficient */
    m_rightFront.configSelectedFeedbackCoefficient(1, Constants.PID_TURN, Constants.kTimeoutMs);
    /* Configure output and sensor direction */
    m_leftFront.setInverted(false);
    m_leftFront.setSensorPhase(true);
    m_rightFront.setInverted(true);
    m_rightFront.setSensorPhase(false);

    /* Set status frame periods to ensure we don't have stale data */

    m_rightFront.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 5, Constants.kTimeoutMs);
    m_rightFront.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 5, Constants.kTimeoutMs);
    m_rightFront.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 5, Constants.kTimeoutMs);
    m_rightFront.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5, Constants.kTimeoutMs);
    m_leftFront.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    /* Configure neutral deadband */
    m_rightFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    m_leftFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    /* Motion Magic Configurations */
    m_rightFront.configMotionAcceleration(1000, Constants.kTimeoutMs);
    // m_rightFront.configMotionCruiseVelocity((int)((10.0 / 600.0) * 4096.0),
    // Constants.kTimeoutMs);
    m_rightFront.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    m_leftFront.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_leftFront.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    m_rightFront.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_rightFront.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

    /* FPID Gains for distance servo */
    m_rightFront.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
    m_rightFront.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
    m_rightFront.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
    m_rightFront.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
    m_rightFront.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
    m_rightFront.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput,
        Constants.kTimeoutMs);
    m_rightFront.configAllowableClosedloopError(Constants.kSlot_Distanc, 0, Constants.kTimeoutMs);

    /* FPID Gains for turn servo */
    m_rightFront.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
    m_rightFront.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
    m_rightFront.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
    m_rightFront.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
    m_rightFront.config_IntegralZone(Constants.kSlot_Turning, (int) Constants.kGains_Turning.kIzone,
        Constants.kTimeoutMs);
    m_rightFront.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput,
        Constants.kTimeoutMs);
    m_rightFront.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if
     * sensor updates are too slow - sensor deltas are very small per update, so
     * derivative error never gets large enough to be useful. - sensor movement is
     * very slow causing the derivative error to be near zero.
     */

    int closedLoopTimeMs = 5;
    m_rightFront.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    m_rightFront.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    m_rightFront.configAuxPIDPolarity(true, Constants.kTimeoutMs);

    zeroSensors();

  }

  public void zeroSensors() {
    m_leftFront.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    m_rightFront.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All Drive Subsystem Encoders are zeroed.\n");
  }

  public void resetGyro() {
    m_navXMP.zeroYaw();
    m_navXMP.resetDisplacement();
    m_navXMP.reset();
    m_rightFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftFront.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
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
    SmartDashboard.putNumber("right pulses", pulses);
    return wheelDistance(pulses);
  }

  public double leftWheelDistance() {
    double pulses = m_leftFront.getSensorCollection().getQuadraturePosition();
    return wheelDistance(pulses);
  }

  public void drive(double left, double right) {

    m_leftFront.set(left);
    m_rightFront.set(right);

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
  }

  @Override
  public void periodic() {

      ramsetePeriodic();

  }

  public Pose2d getPositionOnField () {
    return m_odometry.getPoseMeters();
  }

  public void ramsetePeriodic() {
    var gyroAngle = Rotation2d.fromDegrees(getAngle());
    
    double leftEncoder = m_leftFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;
    double rightEncoder = m_rightFront.getSelectedSensorPosition() / Constants.kEncoderTicksPerMeter;

    m_odometry.update(gyroAngle, leftEncoder, rightEncoder);

  }
}
