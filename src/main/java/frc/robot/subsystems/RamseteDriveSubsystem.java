/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;
import frc.robot.Constants;

public class RamseteDriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.cDriveLeftFront);
  private WPI_TalonSRX m_leftBack = new WPI_TalonSRX(Constants.cDriveLeftBack);
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.cDriveRightFront);
  private WPI_TalonSRX m_rightBack = new WPI_TalonSRX(Constants.cDriveRightBack);

  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);

  DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSystem.
   */
  public RamseteDriveSubsystem() {
    super();
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


    //    public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20, 1023.0/7200.0,  300,  1.00);
    double kI = 1023.0/7200.0;

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

  }

  void zeroSensors() {
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

  /**
   * 
   * @param left  left drive velocity in encoder ticks per 100ms
   * @param right right drive velocity in encoder ticks per 100ms
   */
  public void drive(double left, double right) {

  
    m_leftFront.set(ControlMode.Velocity, left);
    m_rightFront.set(ControlMode.Velocity, right);
  SmartDashboard.putNumber("left velocity", left);
  SmartDashboard.putNumber("right velocity", right);


  }

  public double getAngle() {
    return m_navXMP.getYaw();
  }

  @Override
  public void periodic() {

    var gyroAngle = Rotation2d.fromDegrees(getAngle());
    double leftEncoder = m_leftFront.getSelectedSensorPosition();
    double rightEncoder = m_rightFront.getSelectedSensorPosition();

    Pose2d newPose = m_odometry.update(gyroAngle, leftEncoder, rightEncoder);

    double newPoseAngle = newPose.getRotation().getDegrees();
    double newDistance = newPose.getTranslation().getY();

    SmartDashboard.putNumber("Od angle", newPoseAngle);
    SmartDashboard.putNumber("Od translation y", newDistance);
    SmartDashboard.putNumber("Od left Encoder", leftEncoder);
    SmartDashboard.putNumber("Od right Encoder", rightEncoder);

  }
}
