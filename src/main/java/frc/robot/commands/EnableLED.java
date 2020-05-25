/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.FeedbackSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * An example command that uses an example subsystem.
 */
public class EnableLED extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 
  private FeedbackSubsystem m_feedbackSubsystem;
  boolean m_turnOn = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EnableLED(FeedbackSubsystem subsystem, boolean TurnOn) {


    m_feedbackSubsystem = subsystem;
    m_turnOn = TurnOn;
    addRequirements(m_feedbackSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feedbackSubsystem.enableLED(m_turnOn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


}
