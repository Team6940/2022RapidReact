// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.SwervePathAction;
import frc.robot.auto.actions.TurretAndShooterAction;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallBottomMode extends SequentialCommandGroup {
  /** Creates a new FiveBallBottomMode. */
  public FiveBallBottomMode(SwerveDriveTrain sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory mTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", 2, 2);
    PathPlannerTrajectory mTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", 2, 2);
    PathPlannerTrajectory mTrajectoryThree = PathPlanner.loadPath("FiveBallBottom-3", 2, 2);

    LimelightSubsystem.getInstance().setLightMode(3);//TODO

    addCommands(
      new InstantCommand(() ->     
      sSwerve.ResetOdometry(
        new Pose2d(
          mTrajectoryOne.getInitialState().poseMeters.getTranslation(),
          mTrajectoryOne.getInitialState().holonomicRotation
          )
        )
      ),

      new SwervePathAction(mTrajectoryOne).deadlineWith(new IntakeAction(),new TurretAndShooterAction()),
      new ShootAction().withTimeout(1),
      new WaitCommand(0.5),

      new SwervePathAction(mTrajectoryTwo).deadlineWith(new IntakeAction(),new TurretAndShooterAction()),
      new ShootAction().withTimeout(1),
      new WaitCommand(0.5),

      new SwervePathAction(mTrajectoryThree).deadlineWith(new IntakeAction(),new TurretAndShooterAction()),
      new ShootAction().withTimeout(1)
    );
  }
}