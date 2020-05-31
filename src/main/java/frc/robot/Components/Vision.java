/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

public class Vision extends SubsystemBase {

  // limelight

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry m_tx = table.getEntry("tx"); 
  private NetworkTableEntry m_ty = table.getEntry("ty"); 
  private NetworkTableEntry m_ta = table.getEntry("ta"); 
  private NetworkTableEntry m_ledMode = table.getEntry("ledMode");
  private NetworkTableEntry m_cameraMode = table.getEntry("camMode");


  /**
   * Creates a new DriveSystem.
   */
  public Vision() {
    super();

    enableCameraMode();
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
  m_cameraMode.setNumber(Constants.cLLCameraDriver);
  m_ledMode.setNumber(Constants.cLLLedOff);
 }


 public void enableVisionMode() {
  m_ledMode.setNumber(Constants.cLLLedOn);
  m_cameraMode.setNumber(Constants.cLLCameraVisionProcess);
 }



  @Override
  public void periodic() {
  }
}
