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

import org.opencv.video.KalmanFilter;

import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;


public class Range extends SubsystemBase {

  private TimeOfFlight m_leftTOF = new TimeOfFlight(Constants.kLeftTOF);
  private TimeOfFlight m_rightTOF = new TimeOfFlight(Constants.kRightTOF);

  // construct a default Kalman filter - 
  // stubbing out the Kalman filter code
  // really making it work now
 // private KalmanFilter m_ultrasonicKalmanFilter = new KalmanFilter();

  // 

  //private final AnalogInput m_ultrasonic = new AnalogInput(1);

  private double m_leftLast = -1.0;
  private double m_rightLast = -1.0;

  //private static final double kValueToInches = 0.125;
  // private static final double kValueToInches = (21.0 * 12) / 4095.0; // max value at 21 feet
  private static final double kValueToInches = .052;  //  (41) / 530.0; // max value at 21 feet



  /**
   * Creates a new DriveSystem.
   */
  public Range() {
    super();
    m_leftTOF.setRangingMode(RangingMode.Medium, Constants.kTOFTimingSampling);
    m_rightTOF.setRangingMode(RangingMode.Medium, Constants.kTOFTimingSampling);
    // m_ultrasonic.setAutomaticMode(true);

   // SmartDashboard.putData(m_ultrasonic);
    

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

    // m_kalmanFilter 




  //  NetworkTableEntry ultrasonic = table.getEntry("UltrasonicDistance");
  //  double currentDistance = m_ultrasonic.getValue() * kValueToInches;
  //  ultrasonic.setDouble(currentDistance );
  //  NetworkTableEntry ultrasonicVoltage = table.getEntry("UltrasonicVoltage");
  //  ultrasonicVoltage.setDouble(m_ultrasonic.getValue());


   

    
    



  }

}
