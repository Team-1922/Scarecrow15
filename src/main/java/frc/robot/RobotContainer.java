/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.commands.DriveSquareCommand;
import frc.robot.commands.DriveToDistanceCommand;
import frc.robot.commands.FollowImageCommand;
import frc.robot.commands.EnableLED;
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
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final FeedbackSubsystem m_feedbackSubsystem = new FeedbackSubsystem();

  private final Joystick m_leftJoystick = new Joystick(Constants.cJoyStickLeft);
  private final Joystick m_rightJoystick = new Joystick(Constants.cJoyStickRight);
  private final XboxController m_XBoxController = new XboxController(Constants.cXBoxController);
  private final Vision m_vision = new Vision();
  private final BeamBreak m_beamBreak = new BeamBreak(Constants.kBeamBreak);

  private final TankDriveCommand m_tankDriveCommand = new TankDriveCommand(m_driveSubsystem, m_leftJoystick,m_rightJoystick);

  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

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
 
    m_autoChooser.setDefaultOption("straight", buildAutoCommand(0));  
    m_autoChooser.addOption("serpentine", buildAutoCommand(1));
    m_autoChooser.addOption("Go Home", buildAutoCommand(2));

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

    /*
     * new JoystickButton(m_XBoxController,
     * Constants.cXBoxAButton).toggleWhenPressed(m_followImageCommand);
     * 
     * new JoystickButton(m_XBoxController, Constants.cXBoxBButton).whenPressed(new
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

    // return buildAutoCommand(false);
  }

  public CommandBase buildAutoCommand(int pathOption) {
    // Create a voltage constraint to ensure we don't accelerate too fast

    // m_driveSubsystem.initForRamseteDrive();
    // m_driveSubsystem.zeroPose();

    System.out.println("RobotContainer.buildAutoCommand]");

    BiConsumer<Double, Double> ramseteConsumer = (left, right) -> {
      updateDashboard(left, right);
      m_driveSubsystem.drive(left, right);
    };

    var driveConstraint = new DifferentialDriveKinematicsConstraint(Constants.kDriveKinematics,
        Constants.kAutonomousMaxSpeedMetersPerSecond);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kAutonomousMaxSpeedMetersPerSecond,
        Constants.kAutoMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics).addConstraint(driveConstraint);

    // An example trajectory to follow. All units in meters.
    // Trajectory trajectory = generateStraightPath(config, 2.0); 
    Trajectory trajectory;
    if (0 == pathOption) {
      trajectory = generateStraightPath(config, 2.0); //
    } else if (1 == pathOption ) {
      trajectory = generateSPath(config);
    }
    else {
      trajectory = generateHomePath(config);

    }

    

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_driveSubsystem::getPositionOnField,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, ramseteConsumer,
        m_driveSubsystem);

    return ramseteCommand.andThen(() -> m_driveSubsystem.stop());

  }

  public Trajectory generateStraightPath(TrajectoryConfig config, double distance) {


    Pose2d startPose = m_driveSubsystem.getPositionOnField();

    Rotation2d endRotation = startPose.getRotation();
    Translation2d translation = new Translation2d(distance, 0.0);

    Pose2d endPose = new Pose2d(translation, endRotation);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(startPose, endPose),
        // Pass config
        config);

    // List<Trajectory.State> states = trajectory.getStates();
    // logGeneratedTrajectory(states);

    return trajectory;
  }

  public Trajectory generateHomePath(TrajectoryConfig config) {


    Pose2d startPose = m_driveSubsystem.getPositionOnField();

    Rotation2d endRotation = startPose.getRotation();
    Translation2d translation = new Translation2d(2.0, 0.0);

    Pose2d endPose = new Pose2d(translation, endRotation);
/*
    
    Pose2d startPose = m_driveSubsystem.getPositionOnField();
    Pose2d endPose = m_driveSubsystem.getPositionOnField();
//    Pose2d endPose = m_driveSubsystem.getHomePosition();
*/
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(startPose, endPose),
        // Pass config
        config);

    // List<Trajectory.State> states = trajectory.getStates();
    // logGeneratedTrajectory(states);

    return trajectory;
  }

  public Trajectory generateSPath(TrajectoryConfig config) {

    double distance = 3.0;
    
    Pose2d startPose = m_driveSubsystem.getPositionOnField();

    Rotation2d endRotation = startPose.getRotation();
    Translation2d translation = new Translation2d(distance, 0.0);

    Pose2d endPose = new Pose2d(translation, endRotation);


    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        startPose,
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0.25), new Translation2d(2, -0.25)),
        // End 3 meters straight ahead of where we started, facing forward
        endPose,
        // Pass config
        config);

    // List<Trajectory.State> states = trajectory.getStates();
    // logGeneratedTrajectory(states);
    return trajectory;
  }

  public void updateDashboard(double left, double right) {

    double traveled = m_driveSubsystem.getPositionOnField().getTranslation().getX();

    SmartDashboard.putNumber("xdrive", left);
    SmartDashboard.putNumber("ydrive", right);
    SmartDashboard.putNumber("traveled", traveled);
  }

}
