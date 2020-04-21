/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example command that uses an example subsystem.
 */
public class FollowImageCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry m_tx = table.getEntry("tx"); 
  private NetworkTableEntry m_ty = table.getEntry("ty"); 
  private NetworkTableEntry m_ta = table.getEntry("ta"); 
  private NetworkTableEntry m_ledMode = table.getEntry("ledMode");
  private NetworkTableEntry m_cameraMode = table.getEntry("camMode");



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowImageCommand(DriveSubsystem subsystem) {

    m_driveSubsystem = subsystem;
    m_ledMode.setNumber(Constants.cLLLedOff);
    m_cameraMode.setNumber(Constants.cLLCameraDriver);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledMode.setNumber(Constants.cLLLedOn);
    m_cameraMode.setNumber(Constants.cLLCameraVisionProcess);
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      //Limelight

      SmartDashboard.putNumber("tx", m_tx.getDouble(0.0));
      SmartDashboard.putNumber("ty", m_ty.getDouble(0.0));
      SmartDashboard.putNumber("ta", m_ta.getDouble(0.0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledMode.setNumber(Constants.cLLLedOff);
    m_cameraMode.setNumber(Constants.cLLCameraDriver);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
