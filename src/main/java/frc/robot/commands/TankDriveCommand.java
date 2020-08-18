/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Components.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * An example command that uses an example subsystem.
 */
public class TankDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;
  private final Vision m_vision;  // reference to the vision processing component

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveCommand(DriveSubsystem subsystem, Joystick leftStick, Joystick rightStick, Vision vision) {

    m_driveSubsystem = subsystem;
    m_leftStick = leftStick;
    m_rightStick = rightStick;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_driveSubsystem.initControlSystem();
    m_vision.enableVisionMode();
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d location = m_vision.locationOnField();
    if(location != null)
    {
     m_driveSubsystem.overwriteOdometry(location);
    }

    double left = Constants.kMaxTelopVelocity * deadBand(-m_leftStick.getY());
    double right  = Constants.kMaxTelopVelocity * deadBand(-m_rightStick.getY());

    // the drive system is set up to target speed, so - we want to convert this to a speed which will be a range of full reverse to full forward or ~ -3m/s to 3m/s
   //   m_driveSubsystem.drive(-m_leftStick.getY(),-m_rightStick.getY());
   m_driveSubsystem.drive(left, right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
