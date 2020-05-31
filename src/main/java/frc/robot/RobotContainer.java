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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.DrivePathCommand;
import frc.robot.commands.DriveSquareCommand;
import frc.robot.commands.EnableLED;
import frc.robot.commands.FollowImageCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedbackSubsystem;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Components.BeamBreak;
import frc.robot.Components.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

  private final FeedbackSubsystem m_feedbackSubsystem = new FeedbackSubsystem();

  private final Joystick m_leftJoystick = new Joystick(Constants.cJoyStickLeft);
  private final Joystick m_rightJoystick = new Joystick(Constants.cJoyStickRight);
  private final XboxController m_XBoxController = new XboxController(Constants.cXBoxController);
  private final Vision m_vision = new Vision();
  private final BeamBreak m_beamBreak = new BeamBreak(Constants.kBeamBreak);
  
  private final TankDriveCommand m_tankDriveCommand = new TankDriveCommand(m_driveSubsystem, m_leftJoystick,m_rightJoystick);

  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  FollowImageCommand m_visionServo = new FollowImageCommand(m_driveSubsystem, m_vision);


  /*
   * private final FollowImageCommand m_followImageCommand = new
   * FollowImageCommand(m_driveSubsystem); private final DriveSquareCommand
   * m_DriveSquareCommand = new DriveSquareCommand(m_driveSubsystem,
   * m_ramseteDriveSubsystem);
   */

  /**
   * The container for the robot. Contains subsystems, OI dmevices, and commands.
   */
  public RobotContainer() {

    System.out.println("[RobotContainer]  creating the robot container");
    configureButtonBindings();

  }

  public void robotInit() {

    m_driveSubsystem.initControlSystem();
    m_vision.enableCameraMode();
 
    m_autoChooser.setDefaultOption("straight", DrivePathCommand.buildStraightCommand());  
    m_autoChooser.addOption("s curve", DrivePathCommand.buildSCurveCommand());  
   // m_autoChooser.addOption("Square", DriveSquareCommand.buildDriveSquareCommand());

    SmartDashboard.putData("Autonomous Chooser", m_autoChooser);

  }

  public void autonomousInit(){
    m_driveSubsystem.zeroPose();
  }

  public void teleopInit() {
    m_feedbackSubsystem.enableLED(true);
    m_driveSubsystem.setDefaultCommand(m_tankDriveCommand);
  }

  public void teleopPeriodic() {

    if (m_beamBreak.broken()) {
      if (m_feedbackSubsystem.ledsOn()) {
        CommandScheduler.getInstance().schedule(new EnableLED(m_feedbackSubsystem, false));
      }
    } else {
      if (!m_feedbackSubsystem.ledsOn()) {
        CommandScheduler.getInstance().schedule(new EnableLED(m_feedbackSubsystem, true));
      }
    }

  }

  public void disablePeriodic() {
    m_feedbackSubsystem.enableLED(false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
      new JoystickButton(m_XBoxController, Constants.cXBoxAButton).whileHeld(m_visionServo);
      
     /* new JoystickButton(m_XBoxController, Constants.cXBoxBButton).whenPressed(new
     * DriveToDistanceCommand(m_driveSubsystem, 10));
     * 
     * new JoystickButton(m_XBoxController, Constants.cXBoxYButton).whenPressed(new
     * TurnToHeadingCommand(m_driveSubsystem, 90)); //turns right hopefully
     * 
     * new JoystickButton(m_XBoxController, Constants.cXBoxXButton).whenPressed(new
     * TurnToHeadingCommand(m_driveSubsystem, -90)); //turns left
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_DriveSquareCommand;

    return (Command) m_autoChooser.getSelected();
  }



}
