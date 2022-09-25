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
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeSolState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.auto.actions.SwervePathAction;
import frc.robot.auto.actions.IntakeAndHopperAction;
import frc.robot.commands.Limelight.AutoAim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBallMode2 extends SequentialCommandGroup {
  /** Creates a new SixBallMode. */
  public SixBallMode2(SwerveDriveTrain sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory mSixBallTrajectoryOne = PathPlanner.loadPath("SixBallBottom-1", 2, 2);
    PathPlannerTrajectory mSixBallTrajectoryTwo = PathPlanner.loadPath("SixBallBottom-2", 2, 2);
    PathPlannerTrajectory mSixBallTrajectoryThree = PathPlanner.loadPath("SixBallBottom-3", 2, 2);
    PathPlannerTrajectory mSixBallTrajectoryFour = PathPlanner.loadPath("SixBallBottom-4", 2, 2);
    PathPlannerTrajectory mSixBallTrajectoryFive = PathPlanner.loadPath("SixBallBottom-5", 2, 2);
    PathPlannerTrajectory mSixBallTrajectorySix = PathPlanner.loadPath("SixBallBottom-6", 2, 2);

    addCommands(
      new InstantCommand(() -> LimelightSubsystem.getInstance().setLightMode(3)),

      new InstantCommand(() ->     
      sSwerve.ResetOdometry(
        new Pose2d(
          mSixBallTrajectoryOne.getInitialState().poseMeters.getTranslation(),
          mSixBallTrajectoryOne.getInitialState().holonomicRotation
          )
        )
      ),

      new SwervePathAction(mSixBallTrajectoryOne).deadlineWith(
        new IntakeAndHopperAction()
        ),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(1),
      new WaitCommand(0.5),

      new SwervePathAction(mSixBallTrajectoryTwo).deadlineWith(
        new IntakeAndHopperAction()
        ),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(1),

      new SwervePathAction(mSixBallTrajectoryThree).deadlineWith(
        new IntakeAndHopperAction()
        ),
      new WaitCommand(0.5),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(1),

      new SwervePathAction(mSixBallTrajectoryFour).deadlineWith(
        new IntakeAndHopperAction()
        ),
      new AutoAim().withTimeout(1),

      new SwervePathAction(mSixBallTrajectoryFive).deadlineWith(
        new IntakeAndHopperAction()
        ),
      new WaitCommand(0.5),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(1),

      new SwervePathAction(mSixBallTrajectorySix).deadlineWith(
        new IntakeAndHopperAction()
        ),
      new WaitCommand(0.5),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(1),

      new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.CLOSE)),
      new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.OFF)),
      new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.OFF))

    );
  }
}
