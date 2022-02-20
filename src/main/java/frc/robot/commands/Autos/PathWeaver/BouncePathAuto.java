package frc.robot.commands.Autos.PathWeaver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.FollowTrajectory;
import frc.robot.commands.Autos.StopDrive;
import frc.robot.subsystems.SwerveDriveTrain;

/**
 * Instructs the robot to follow each of the bounce path's legs in sequence
 */
public class BouncePathAuto extends SequentialCommandGroup {
  public BouncePathAuto (SwerveDriveTrain drive) {
    addCommands(
            new FollowTrajectory(drive, "bounceLeg1.wpilib.json"),
            new FollowTrajectory(drive, "bounceLeg2.wpilib.json"),
            new FollowTrajectory(drive, "bounceLeg3.wpilib.json"),
            new FollowTrajectory(drive, "bounceLeg4.wpilib.json"),
            new StopDrive()
    );
  }
}
