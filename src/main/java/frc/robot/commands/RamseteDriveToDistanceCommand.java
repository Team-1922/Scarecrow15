/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * An example command that uses an example subsystem.
 */
public class RamseteDriveToDistanceCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RamseteDriveSubsystem m_driveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem    The subsystem used by this command.
   * @param distanceFeet The distance in feet this command should drive. Must be
   *                     positive
   */
  public RamseteDriveToDistanceCommand(RamseteDriveSubsystem subsystem, double distanceFeet) {

    m_driveSubsystem = subsystem;
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
    setSpeed();
  }

  public void setSpeed() {

    // Creating my kinematics object: track width of 27 inches
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(18.0));

    double backToStraight = Units.degreesToRadians(m_driveSubsystem.getAngle());

    // Example chassis speeds: 2 meters per second linear velocity,
    // 1 radian per second angular velocity.
    var chassisSpeeds = new ChassisSpeeds(0.001, 0, 0);

    // Convert to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;

    // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    SmartDashboard.putNumber("left Velocity", leftVelocity);
    SmartDashboard.putNumber("right Velocity", rightVelocity);

    // meters/sec * (1sec/10 (100ms)) * (39.37 in/meter) * (1revolution/12.57in) *
    // (4096 ticks/rev) = ticks/100ms
    double leftVelocityInInches = Units.metersToInches(leftVelocity);
    double leftVelocityTalonUnits = leftVelocityInInches * (1.0 / 10.0) * (1.0 / 12.57) * 4096.0;

    double rightVelocityInInches = Units.metersToInches(rightVelocity);
    double rightVelocityTalonUnits = rightVelocityInInches * (1.0 / 10.0) * (1.0 / 12.57) * 4096.0;

    m_driveSubsystem.drive(leftVelocityTalonUnits, rightVelocityTalonUnits);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
