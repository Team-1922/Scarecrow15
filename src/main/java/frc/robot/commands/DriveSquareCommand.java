/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveSquareCommand extends SequentialCommandGroup {
  /**
   * Creates a new DriveSquareCommand.
   */
  public DriveSquareCommand(DriveSubsystem subsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ResetGyroCommand(subsystem),
      new WaitCommand(1),
      // new DriveToDistanceCommand(subsystem, 30),
      // new RamseteDriveToDistanceCommand(subsystem, 10),
      new WaitCommand(2)
    //  new TurnToHeadingCommand(subsystem, 90)
     /* new DriveToDistanceCommand(subsystem, 2),
      new TurnToHeadingCommand(subsystem, 180),
      new DriveToDistanceCommand(subsystem, 2),
      new TurnToHeadingCommand(subsystem, 270),
      new DriveToDistanceCommand(subsystem, 2),
      new TurnToHeadingCommand(subsystem, 0)*/
    );
  }
}
