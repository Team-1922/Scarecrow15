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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;



import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  
  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX (Constants.cDriveLeftFront);
  private WPI_TalonSRX m_leftBack = new WPI_TalonSRX (Constants.cDriveLeftBack);
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX (Constants.cDriveRightFront);
  private WPI_TalonSRX m_rightBack = new WPI_TalonSRX (Constants.cDriveRightBack);

  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);

  
  // set up the limelight
  



  /**
   * Creates a new DriveSystem.
   */
  public DriveSubsystem() {
    super();

    m_rightFront.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightFront.setSelectedSensorPosition(0, 0, 10);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightFront.config_kF(0, 0.3);
    m_rightFront.config_kP(0, 0);
    m_rightFront.selectProfileSlot(0, 0);
    m_rightFront.configMotionCruiseVelocity((int)((1000.0 / 600.0) * 4096.0)); // 1000 rev/min * 1min/600 (100ms) * 4096 tick/rev = ticks/100ms
    m_rightFront.configMotionAcceleration(2275);
    m_rightFront.setSensorPhase(false);
    m_rightFront.setInverted(true);


    m_leftFront.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_leftFront.setSelectedSensorPosition(0, 0, 10);
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftFront.config_kF(0, 0.3);
    m_leftFront.config_kP(0, 0);
    m_leftFront.selectProfileSlot(0, 0);
    m_leftFront.configMotionCruiseVelocity(1000 / 600 * 4096); // 1000 rev/min * 1min/600 (100ms) * 4096 tick/rev = ticks/100ms
    m_leftFront.configMotionAcceleration(2275);

    m_rightBack.set(ControlMode.Follower, m_rightFront.getDeviceID());
    m_rightBack.setInverted(InvertType.FollowMaster);
    m_leftBack.set(ControlMode.Follower, m_leftFront.getDeviceID());
    m_leftBack.setInverted(InvertType.FollowMaster);


  }
  public void resetGyro()
  {
    m_navXMP.zeroYaw();
    m_navXMP.resetDisplacement();
    m_navXMP.reset();
  }

  public void stop()
  {
    m_leftFront.stopMotor();
    m_rightFront.stopMotor();
  }

  public void drive (double left, double right)
  {
   
    if (Math.abs(left) > Constants.cDriveBaseDeadband) {
      m_leftFront.set(left);
    }

    if (Math.abs(right) > Constants.cDriveBaseDeadband){
      m_rightFront.set(right);
    }
    
  }

  public int getLeftError()
  {
    return m_leftFront.getClosedLoopError();
  }

  public int getRightError()
  {
    return m_rightFront.getClosedLoopError();
  }

  public void driveToTarget(double rightTarget, double leftTarget)
  {
    m_rightFront.set(ControlMode.MotionMagic, rightTarget);
    m_leftFront.set(ControlMode.MotionMagic, leftTarget);
  }
  
  public double getLeftEncoder() {
    return m_leftFront.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return m_rightFront.getSelectedSensorPosition();
  }

  public double getAngle() {
    return m_navXMP.getYaw();
  }

  
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber ("left encoder", getLeftEncoder());
    SmartDashboard.putNumber ("right encoder", getRightEncoder());
    SmartDashboard.putNumber ("heading angle", getAngle());
  }
}
