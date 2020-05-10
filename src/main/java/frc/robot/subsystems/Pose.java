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

import java.lang.Math;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



import frc.robot.Constants;
import frc.robot.Components.BeamBreak;

public class Pose extends SubsystemBase {

  
  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);
  private PigeonIMU m_imu = new PigeonIMU (Constants.kIMU);

  private TimeOfFlight m_leftTOF = new TimeOfFlight(Constants.kLeftTOF);
  private TimeOfFlight m_rightTOF = new TimeOfFlight(Constants.kRightTOF);

  BeamBreak m_beamBreak = new BeamBreak(0);

  // limelight

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry m_tx = table.getEntry("tx"); 
  private NetworkTableEntry m_ty = table.getEntry("ty"); 
  private NetworkTableEntry m_ta = table.getEntry("ta"); 
  private NetworkTableEntry m_ledMode = table.getEntry("ledMode");
  private NetworkTableEntry m_cameraMode = table.getEntry("camMode");
  private NetworkTableEntry m_pipeline = table.getEntry("pipeline");


  /**
   * Creates a new DriveSystem.
   */
  public Pose() {
    super();
    m_leftTOF.setRangingMode(RangingMode.Short, Constants.kTOFTimingSampling);
    m_rightTOF.setRangingMode(RangingMode.Short, Constants.kTOFTimingSampling);

    enableCameraMode();

    register(); // let the command scheduler know this exists and hook it into the loop
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

 public void enableCameraMode() {
   System.out.println("[Pose]  enable Camera mode");

 // m_cameraMode.setNumber(Constants.cLLCameraDriver);
 // m_ledMode.setNumber(Constants.cLLLedOff);
  m_pipeline.setNumber(1);
 }


 public void visionMode() {
  m_ledMode.setNumber(Constants.cLLLedOn);
  m_cameraMode.setNumber(Constants.cLLCameraVisionProcess);
 }


 public void IMUInit() {
  m_imu.setYaw(0);
 }

  public double getAngle() {
    return m_navXMP.getAngle();
  }

 
   /**
   * returns the distance in millimeters to an obstacle that the TOF sensor reports 
   *
   */
  public double getLeftTOFDistance() {
    return m_leftTOF.getRange();
  }
  /**
   * returns the distance in millimeters to an obstacle that the TOF sensor reports 
   *
   */
  public double getRightTOFDistance() {
    return m_rightTOF.getRange();
  }

  public boolean beamBroken() {
    return m_beamBreak.broken();

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber ("Pose heading", getAngle());
    m_beamBreak.periodic();
    enableCameraMode();

    double[] ypr = new double[3];
    m_imu.getYawPitchRoll(ypr);


    double compassHeading = m_imu.getAbsoluteCompassHeading();
    SmartDashboard.putNumber ("IMU angle", ypr[0]);




    SmartDashboard.putNumber("tx", tx());
    SmartDashboard.putNumber("ty", ty());
    SmartDashboard.putNumber("ta", ta());

    SmartDashboard.putNumber ("Left TOF", getLeftTOFDistance());
    SmartDashboard.putNumber ("Right TOF", getRightTOFDistance());

    // will get some blinking of the widget depending on the sampling time
   // SmartDashboard.putBoolean("Left valid", m_leftTOF.isRangeValid());
   // SmartDashboard.putBoolean("Right valid", m_rightTOF.isRangeValid());
 
  }
}
