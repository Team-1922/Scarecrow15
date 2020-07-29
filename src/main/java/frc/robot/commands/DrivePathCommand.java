/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DrivePathCommand extends RamseteCommand {
  /**
   * Creates a new DrivePathCommand.
   */

  static BiConsumer<Double, Double> ramseteConsumer = (left, right) -> {
    DriveSubsystem.getInstance().drive(left, right);
  };

  static RamseteController m_controller = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);

  public DrivePathCommand(Trajectory path) {

    super(path, DriveSubsystem.getInstance()::getPositionOnField, m_controller, Constants.kDriveKinematics,
        ramseteConsumer, DriveSubsystem.getInstance());

    addRequirements(DriveSubsystem.getInstance());
  }

  /* 
  static public CommandBase buildHomeCommand() {
    Trajectory path = buildHomePath();
    return new DrivePathCommand(path).andThen(() -> DriveSubsystem.getInstance().stop());

  }
*/

 /*
  static public Trajectory buildHomePath() {

    Pose2d startPose = DriveSubsystem.getInstance().getPositionOnField();
    Pose2d endPose = DriveSubsystem.getInstance().getHomePosition();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(startPose, endPose),
        // Pass config
        Constants.kConfig);

    return trajectory;
  }
*/
  static public CommandBase buildStraightCommand() {

    Trajectory path = buildStraightPath(2);
    return new DrivePathCommand(path).andThen(() -> DriveSubsystem.getInstance().stop());
  }



  static public Trajectory buildStraightPath(double distance) {

    Pose2d startPose = DriveSubsystem.getInstance().getPositionOnField();

    Rotation2d endRotation = startPose.getRotation();
    Translation2d translation = new Translation2d(distance, 0.0);
    Transform2d transform = new Transform2d(translation, endRotation);
    
    Pose2d endPose = startPose.plus(transform);


    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(startPose, endPose),
        // Pass config
        Constants.kConfig);

    return trajectory;
  }

  static public CommandBase buildStraightToGoalCommand()
  {
    Pose2d currentPosition = DriveSubsystem.getInstance().getPositionOnField();
    Trajectory path = buildStraightPathToGoal(currentPosition);
    return new DrivePathCommand(path).andThen(() -> DriveSubsystem.getInstance().stop());
  }

  static public Trajectory buildStraightPathToGoal(Pose2d currentPosition)
  {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(currentPosition, Constants.kPowerPortShootingLocation), Constants.kConfig);
    return trajectory;
  }

  static public CommandBase buildSCurveCommand() {

    Trajectory path = generateSPath();
    return new DrivePathCommand(path).andThen(() -> DriveSubsystem.getInstance().stop());

  }

  static public Trajectory generateSPath() {

    double distance = 3.0;

    Pose2d startPose = DriveSubsystem.getInstance().getPositionOnField();

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
        Constants.kConfig);

    // List<Trajectory.State> states = trajectory.getStates();
    // logGeneratedTrajectory(states);
    return trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    double traveled = DriveSubsystem.getInstance().getPositionOnField().getTranslation().getX();

    SmartDashboard.putNumber("traveled", traveled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
