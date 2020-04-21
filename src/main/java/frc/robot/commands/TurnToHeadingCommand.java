/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private double m_heading;
  private boolean m_turnRight;
  private static double errorThreshold = 1;
  /**
   * Creates a new TurnToHeadingCommand.
   */
  public TurnToHeadingCommand(DriveSubsystem subsystem, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_subsystem = subsystem;
    m_heading = heading;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentHeading = m_subsystem.getAngle();
    double headingDifference = m_heading - currentHeading;
    m_turnRight = headingDifference > 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_turnRight) {
      m_subsystem.drive(0.3, -0.3);
    }
    else{
      m_subsystem.drive(-0.3, 0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentHeading = m_subsystem.getAngle();
    double headingDifference = Math.abs(m_heading - currentHeading);
    SmartDashboard.putNumber("headingDifference", headingDifference);
    return headingDifference < errorThreshold;
  }
}
