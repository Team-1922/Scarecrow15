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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;



import frc.robot.Constants;

public class Pose extends SubsystemBase {

  
  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);

  private TimeOfFlight m_leftTOF = new TimeOfFlight(Constants.kLeftTOF);
  private TimeOfFlight m_rightTOF = new TimeOfFlight(Constants.kRightTOF);

  /**
   * Creates a new DriveSystem.
   */
  public Pose() {
    super();
    
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

  @Override
  public void periodic() {

    SmartDashboard.putNumber ("Pose heading", getAngle());
    SmartDashboard.putNumber ("Left TOF", getLeftTOFDistance());
    SmartDashboard.putNumber ("Right TOF", getRightTOFDistance());
  }
}
