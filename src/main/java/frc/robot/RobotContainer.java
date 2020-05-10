/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.commands.DriveSquareCommand;
import frc.robot.commands.DriveToDistanceCommand;
import frc.robot.commands.FollowImageCommand;
import frc.robot.commands.EnableLED;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedbackSubsystem;
import frc.robot.subsystems.Pose;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final FeedbackSubsystem m_feedbackSubsystem = new FeedbackSubsystem();
  private final Pose m_pose = new Pose();

 

  private final Joystick m_leftJoystick = new Joystick(Constants.cJoyStickLeft);
  private final Joystick m_rightJoystick = new Joystick(Constants.cJoyStickRight);
  private final XboxController m_XBoxController = new XboxController(Constants.cXBoxController);

  private final TankDriveCommand m_tankDriveCommand = new TankDriveCommand(m_driveSubsystem, m_leftJoystick, m_rightJoystick);
  private final FollowImageCommand m_followImageCommand = new FollowImageCommand(m_driveSubsystem);
  private final DriveSquareCommand m_DriveSquareCommand = new DriveSquareCommand(m_driveSubsystem);

  /**
   * The container for the robot.  Contains subsystems, OI dmevices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_tankDriveCommand);
   // m_pose.initialize();
    System.out.println("[RobotContainer]  creating the robot container");
    configureButtonBindings();
    m_pose.enableCameraMode();
  }



public void IMUInit(){
  m_pose.IMUInit();
}

public void teleopInit()
{
  m_feedbackSubsystem.enableLED(true);
  IMUInit();
}

public void teleopPeriodic(){

  if (m_pose.beamBroken()) {
    if( m_feedbackSubsystem.ledsOn()) {
      CommandScheduler.getInstance().schedule(new EnableLED(m_feedbackSubsystem, false));
    }
  }
  else {
    if (!m_feedbackSubsystem.ledsOn()) {
      CommandScheduler.getInstance().schedule(new EnableLED(m_feedbackSubsystem, true));
    }
  }

}

public void disablePeriodic() {
  m_feedbackSubsystem.enableLED(false);
}


  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
    new JoystickButton(m_XBoxController, Constants.cXBoxAButton).toggleWhenPressed(m_followImageCommand);

    new JoystickButton(m_XBoxController, Constants.cXBoxBButton).whenPressed(new DriveToDistanceCommand(m_driveSubsystem, 10));

    new JoystickButton(m_XBoxController, Constants.cXBoxYButton).whenPressed(new TurnToHeadingCommand(m_driveSubsystem, 90)); //turns right hopefully

    new JoystickButton(m_XBoxController, Constants.cXBoxXButton).whenPressed(new TurnToHeadingCommand(m_driveSubsystem, -90)); //turns left

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_DriveSquareCommand;
  }
}
