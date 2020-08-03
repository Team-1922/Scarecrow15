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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;



/**
 * An example command that uses an example subsystem.
 */
public class PositionResetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final Vision m_vision;  // reference to the vision processing component

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionResetCommand(DriveSubsystem subsystem, Vision vision) {

    m_driveSubsystem = subsystem;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.enableVisionMode();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*

The similar triangle thing may be a false start. It's a nice solution for when you're pointed directly at the thing you're using to range but as soon as you're not it all falls apart.

I think you can do a bit better by using tx and heading to get the angle to the target off the vertical and then use your distance as the hypotenuse of the triangle. Then you just get (remembering that our x,y directions are flipped from what you'd expect) xDistanceFromTarget = distanceToTarget * sin(tx + heading) yDistanceFromTarget = distanceToTarget * cos(tx + heading)

    //estimatedDistance is our distance to the target based on where our odometry thinks we are
// we'll move along our heading vector some distance moveDistance 
// tx_predicted is the tx we think we should be getting from limelight based on where our odometry thinks we are
// actualDistance is the distance the limelight reports
// tx is the tx the limelight reports
moveDistance = (estimatedDistance - actualDistance*cos(tx - tx_predicted)) / cos(tx_predicted)

Other approach:

We have our distance (estimated Distance) and an angle (tx) to a known point (the target). Rather than calculating how to change our current position
just translate from the target according to that vector.

    */



    //  Translation2d targetPosition = new Translation2d(Units.inchesToMeters(129), Units.inchesToMeters(51));
    Translation2d targetPosition = new Translation2d(Constants.kPowerPort.getTranslation().getX(), Constants.kPowerPort.getTranslation().getY());

    double limelightDistanceToCenterOfRobot = Units.inchesToMeters(m_vision.getDistanceFromTarget()) + 12;
    double angleToTarget = m_vision.tx();

    Pose2d currentEstimatedPosition = m_driveSubsystem.getPositionOnField();
    Rotation2d currentAngle = currentEstimatedPosition.getRotation(); // we'll assume this is correct

    Translation2d baseVector = new Translation2d(limelightDistanceToCenterOfRobot, 0);
    Rotation2d rotation = Rotation2d.fromDegrees(-(currentAngle.getDegrees() - angleToTarget + 180));
    Translation2d targetToRobot = baseVector.rotateBy(rotation);

    Translation2d robotPosition = targetPosition.plus(targetToRobot);

    SmartDashboard.putNumber("guessed X", Units.metersToInches(robotPosition.getX()));
    SmartDashboard.putNumber("guessed Y", Units.metersToInches(robotPosition.getY()));


    // Corner of the carpet is (0,0)
    // Length of carepet is 129in
    // Width of carpet is 94in
    // Target is at 50in 
    // Target is at (50in, 129in)
    // Target is 39in wide & 17in high
    
    m_driveSubsystem.overwriteOdometry(new Pose2d(robotPosition, currentAngle), currentAngle);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.enableCameraMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
