/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToDistanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private int m_leftEncoderStart;
  private int m_rightEncoderStart;
  private int m_leftEncoderTarget;
  private int m_rightEncoderTarget;
  private int m_startEncoderDiff;
  private double m_distanceFeet;

  private boolean m_executed = false;

  private int kErrThreshold = 40;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param distanceFeet The distance in feet this command should drive. Must be positive
   */
  public DriveToDistanceCommand(DriveSubsystem subsystem, double distanceFeet) {

    m_driveSubsystem = subsystem;
    m_distanceFeet = distanceFeet;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_driveSubsystem.initForTalonController();
    
      m_rightEncoderStart = m_driveSubsystem.getEncoderSum();
      m_startEncoderDiff = m_driveSubsystem.getEncoderDifference();
      int encoderTargetDistance = (int)(Constants.ticksPerFoot * m_distanceFeet);
      m_rightEncoderTarget = m_rightEncoderStart + encoderTargetDistance;
      SmartDashboard.putNumber("sensor 0 target", m_rightEncoderTarget);
      SmartDashboard.putNumber("sensor 1 target", m_startEncoderDiff);
      m_executed = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_executed) {
      m_driveSubsystem.driveToTarget(m_rightEncoderTarget, m_startEncoderDiff);
      m_executed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_driveSubsystem.stop();
      m_executed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    int error = m_rightEncoderTarget - m_driveSubsystem.getEncoderSum();
    SmartDashboard.putNumber("error", error);


   // boolean leftInRange =  m_leftEncoderTarget -  m_driveSubsystem.getLeftEncoder() < kErrThreshold;
    boolean rightInRange = error < kErrThreshold;
    SmartDashboard.putBoolean("IsFinished", rightInRange);
   // boolean bothInRange = leftInRange && rightInRange;
   return rightInRange;
  }
}
