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
  private double m_leftEncoderStart;
  private double m_rightEncoderStart;
  private double m_leftEncoderTarget;
  private double m_rightEncoderTarget;
  private double m_distanceFeet;

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
      m_leftEncoderStart = m_driveSubsystem.getLeftEncoder();
      m_rightEncoderStart = m_driveSubsystem.getRightEncoder();
      double encoderTargetDistance = Constants.ticksPerFoot * m_distanceFeet;
      m_leftEncoderTarget = m_leftEncoderStart + encoderTargetDistance;
      m_rightEncoderTarget = m_rightEncoderStart + encoderTargetDistance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_driveSubsystem.driveToTarget(m_rightEncoderTarget, m_leftEncoderTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean leftInRange =  m_leftEncoderTarget -  m_driveSubsystem.getLeftEncoder() < kErrThreshold;
    boolean rightInRange = m_rightEncoderTarget -  m_driveSubsystem.getRightEncoder() < kErrThreshold;

    boolean bothInRange = leftInRange && rightInRange;
    return bothInRange;
  }
}
