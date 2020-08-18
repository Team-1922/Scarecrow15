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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;


/**
 * An example command that uses an example subsystem.
 */
public class FollowImageCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final Vision m_vision;  // reference to the vision processing component
  private double pGain = 0.03;
  private double dGain = 0.00;
  private double lastError = 0.0;
  private final Joystick m_leftStick;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowImageCommand(DriveSubsystem subsystem, Vision vision, Joystick leftStick) {

    m_driveSubsystem = subsystem;
    m_vision = vision;
    m_leftStick = leftStick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Constants.kMaxTelopVelocity * deadBand(-m_leftStick.getY());
      //Limelight
      double xError = m_vision.tx();
      if(xError < 2 && xError > -2)
      {
        m_driveSubsystem.drive(speed, speed);
        return;
      }
      double mult = 1.0;
      if(xError < 0)
      {
          mult = -1.0;
      }
      //we need to turn that many degrees
      double dError = xError - lastError;
      double outputSpeed = (mult * 0.45) + xError * pGain + dError * dGain;
      if(Math.abs(outputSpeed) > 1)
      {
        outputSpeed = 1.0 * Math.abs(outputSpeed)/outputSpeed;
      }

      SmartDashboard.putNumber("outputSpeed", outputSpeed);

      m_driveSubsystem.drive(speed + outputSpeed, speed - outputSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  double deadBand(double value) {

    double val = value * Math.abs(value);  // try to smooth it out a little
    if (Math.abs(val) < 0.05) {
      return 0.0;
    }
    else {
      return val;
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
