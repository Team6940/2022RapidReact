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
import frc.robot.Constants;
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
    PathPlannerTrajectory mFiveBallTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", 2, 2);
    PathPlannerTrajectory mFiveBallTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", 2, 2);
    PathPlannerTrajectory mFiveBallTrajectoryThree = PathPlanner.loadPath("FiveBallBottom-3", 2, 2);
    PathPlannerTrajectory mFiveBallTrajectoryFour = PathPlanner.loadPath("FiveBallBottom-4", 2, 2);
    PathPlannerTrajectory mFiveBallTrajectoryFive = PathPlanner.loadPath("FiveBallBottom-5", 2, 2);
    addCommands(
      new InstantCommand(() -> LimelightSubsystem.getInstance().setLightMode(3)),

      new InstantCommand(() ->     
      sSwerve.ResetOdometry(
        new Pose2d(
          mFiveBallTrajectoryOne.getInitialState().poseMeters.getTranslation(),
          mFiveBallTrajectoryOne.getInitialState().holonomicRotation
          )
        )
      ),

      new SwervePathAction(mFiveBallTrajectoryOne).deadlineWith(new IntakeAction(Constants.vSwitchIntake),new TurretAndShooterAction()),
      new WaitCommand(0.5),

      new SwervePathAction(mFiveBallTrajectoryTwo).deadlineWith(new IntakeAction(Constants.vSwitchIntake),new TurretAndShooterAction()),
      new ShootAction().withTimeout(1),
      //new WaitCommand(0.5),

      new SwervePathAction(mFiveBallTrajectoryThree).deadlineWith(new IntakeAction(Constants.vSwitchIntake),new TurretAndShooterAction()),
      new WaitCommand(0.5),
      new ShootAction().withTimeout(1),

      new SwervePathAction(mFiveBallTrajectoryFour).deadlineWith(new IntakeAction(Constants.vSwitchIntake),new TurretAndShooterAction()),
      new WaitCommand(2),
      //new WaitCommand(0.5),

      new SwervePathAction(mFiveBallTrajectoryFive).deadlineWith(new IntakeAction(Constants.vSwitchIntake),new TurretAndShooterAction()),
      new ShootAction().withTimeout(1)
    );
  }
}
