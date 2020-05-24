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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;

import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * An example command that uses an example subsystem.
 */
public class RamseteDriveToDistanceCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_driveSubsystem;

  double m_targetDistance = 0.0;
  Pose2d m_goalPosition; // = new Pose2d();


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem      The subsystem used by this command.
   * @param distanceMeters The distance in meters this command should drive. Must
   *                       be positive
   */
  public RamseteDriveToDistanceCommand(DriveSubsystem subsystem, double distanceMeters) {

    m_driveSubsystem = subsystem;
    m_targetDistance = distanceMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // m_driveSubsystem.zeroSensors();
    Translation2d translation = new Translation2d(m_targetDistance, 0.0);
    Rotation2d rotation = new Rotation2d(0.0);
    Transform2d transformation = new Transform2d(translation, rotation);

    m_goalPosition = m_driveSubsystem.getPositionOnField().transformBy(transformation);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setSpeed();
  }

  
  public void setSpeed() {

    double backToStraight = Units.degreesToRadians(m_driveSubsystem.getAngle());

    // Example chassis speeds: 2 meters per second linear velocity,
    // 1 radian per second angular velocity.
    var chassisSpeeds = new ChassisSpeeds(0.001, 0, 0);

  
    // Convert to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);

    // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond * 0.1; // velocity per 100 ms

    // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond * 0.1; // velocity per 100 ms

    SmartDashboard.putNumber("left Velocity", leftVelocity);
    SmartDashboard.putNumber("right Velocity", rightVelocity);

    double leftVelocityInTicks = Constants.kEncoderTicksPerMeter * leftVelocity;
    double rightVelocityInTicks = Constants.kEncoderTicksPerMeter * rightVelocity;

    m_driveSubsystem.drive(leftVelocityInTicks, rightVelocityInTicks);

    /*
     * // meters/sec * (1sec/10 (100ms)) * (39.37 in/meter) * (1revolution/12.57in)
     * * // (4096 ticks/rev) = ticks/100ms double leftVelocityInInches =
     * Units.metersToInches(leftVelocity); double leftVelocityTalonUnits =
     * leftVelocityInInches * (1.0 / 10.0) * (1.0 / 12.57) * 4096.0;
     * 
     * double rightVelocityInInches = Units.metersToInches(rightVelocity); double
     * rightVelocityTalonUnits = rightVelocityInInches * (1.0 / 10.0) * (1.0 /
     * 12.57) * 4096.0;
     * 
     * m_driveSubsystem.drive(leftVelocityTalonUnits, rightVelocityTalonUnits);
     */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    Pose2d currentPos = m_driveSubsystem.getPositionOnField();

    Transform2d trans = currentPos.minus(m_goalPosition);

    SmartDashboard.putNumber("Distance", trans.getTranslation().getX());

    if (Math.abs(trans.getTranslation().getX()) < 0.1) {
      SmartDashboard.putBoolean("IsFinished", true);
      return true;
    } else {
      SmartDashboard.putBoolean("IsFinished", false);
      return false;
    }

    /*
     * m_driveSubsystem.getPositionOnField() - m_goalPosition.
     * 
     * double distanceTraveled = m_driveSubsystem.rightWheelDistance();
     * 
     * SmartDashboard.putNumber("Distance", distanceTraveled);
     * 
     * double epsilon = 0.1; // meter
     * 
     * boolean rightInRange = Math.abs(distanceTraveled - m_targetDistance) <
     * epsilon;
     * 
     * SmartDashboard.putBoolean("IsFinished", rightInRange); // boolean bothInRange
     * = leftInRange && rightInRange; return rightInRange;
     */

  }

 


}
