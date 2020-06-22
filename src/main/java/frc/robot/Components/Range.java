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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;

public class Range extends SubsystemBase {

  private TimeOfFlight m_leftTOF = new TimeOfFlight(Constants.kLeftTOF);
  private TimeOfFlight m_rightTOF = new TimeOfFlight(Constants.kRightTOF);

  private double m_leftLast = -1.0;
  private double m_rightLast = -1.0;

  /**
   * Creates a new DriveSystem.
   */
  public Range() {
    super();
    m_leftTOF.setRangingMode(RangingMode.Medium, Constants.kTOFTimingSampling);
    m_rightTOF.setRangingMode(RangingMode.Medium, Constants.kTOFTimingSampling);


    register();

 }

   /**
   * returns the distance in millimeters to an obstacle that the TOF sensor reports 
   *
   */
  public double getLeftTOFDistance() {

    if (m_leftTOF.isRangeValid()) {
      m_leftLast = m_leftTOF.getRange() * 0.001; // convert to meters.
    }

    return m_leftLast;
    
  }
  /**
   * returns the distance in millimeters to an obstacle that the TOF sensor reports 
   *
   */
  public double getRightTOFDistance() {

    if (m_rightTOF.isRangeValid()) {
      m_rightLast = m_rightTOF.getRange() * .001; // convert to meters
    }

    return m_rightLast;
    
  }


  @Override
  public void periodic() {

   // private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");
    NetworkTableEntry left = table.getEntry("TOFLeft");
    left.setDouble(Units.metersToInches(getLeftTOFDistance()));

    NetworkTableEntry right = table.getEntry("TOFRight");
    right.setDouble(Units.metersToInches(getRightTOFDistance()));




  }

}
