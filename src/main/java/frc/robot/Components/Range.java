/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.Constants;

public class Range extends SubsystemBase {

  private TimeOfFlight m_leftTOF = new TimeOfFlight(Constants.kLeftTOF);
  private TimeOfFlight m_rightTOF = new TimeOfFlight(Constants.kRightTOF);


  /**
   * Creates a new DriveSystem.
   */
  public Range() {
    super();
    m_leftTOF.setRangingMode(RangingMode.Short, Constants.kTOFTimingSampling);
    m_rightTOF.setRangingMode(RangingMode.Short, Constants.kTOFTimingSampling);

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
  }
}
