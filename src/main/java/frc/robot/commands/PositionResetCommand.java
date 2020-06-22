/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Components.Vision;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;



/**
 * An example command that uses an example subsystem.
 */
public class PositionResetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final Vision m_vision;  // reference to the vision processing component

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionResetCommand(DriveSubsystem subsystem, Vision vision) {

    m_driveSubsystem = subsystem;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.enableVisionMode();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /*
    double tx = m_vision.tx();
    double ty = m_vision.ty();
    double thor = m_vision.thor();
    double tvert = m_vision.tvert();
    double ts = m_vision.ts();
    //double ta = m_vision.ta();

    double targetHeight = 17;
    double targetDistanceFromGround = 3;
    double centerOfTarget = targetDistanceFromGround + (targetHeight/2.0);
    double centerOfLowTarget = 3.5;
    double limelightDistanceFromGround = 8;
    double tyRadians = Units.degreesToRadians(ty);
    double limelightAngle = Units.degreesToRadians(-3.5);
    double distanceToTarget = (centerOfLowTarget - limelightDistanceFromGround) / Math.tan(tyRadians + limelightAngle);


    SmartDashboard.putNumber("DistanceToTarget (in)", distanceToTarget);
    Pose2d newEstimatedPose;
    Rotation2d newEstimatedAngle;


    // Corner of the carpet is (0,0)
    // Length of carepet is 129in
    // Width of carpet is 94in
    // Target is at 50in 
    // Target is at (50in, 129in)
    // Target is 39in wide & 17in high


    //  m_driveSubsystem.updateOdometry(newEstimatedPose, newEstimatedAngle);

      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
      SmartDashboard.putNumber("thor", thor);
      SmartDashboard.putNumber("tvert", tvert);
      SmartDashboard.putNumber("ts", ts);
      */
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.enableCameraMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
