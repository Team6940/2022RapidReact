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
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.auto.actions.SwervePathAction;
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.IntakeAndHopperAction;
import frc.robot.auto.actions.TurretAction;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeSolState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallMode extends SequentialCommandGroup {
  /** Creates a new TwoBallMode. */
  public TwoBallMode(SwerveDriveTrain sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory mTwoBallTrajectoryOne = PathPlanner.loadPath("TwoBall-1", 4, 4);
    PathPlannerTrajectory mTwoBallTrajectoryTwo = PathPlanner.loadPath("TwoBall-2", 4, 4);


    addCommands(
      new InstantCommand(() -> LimelightSubsystem.getInstance().setLightMode(3)),

      new InstantCommand(() ->     
      sSwerve.ResetOdometry(
        new Pose2d(
          mTwoBallTrajectoryOne.getInitialState().poseMeters.getTranslation(),
          mTwoBallTrajectoryOne.getInitialState().holonomicRotation
          )
        )
      ),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),      
      new SwervePathAction(mTwoBallTrajectoryOne).deadlineWith(
        new IntakeAndHopperAction(),
        new TurretAction()
        ),
      new WaitCommand(0.5),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mTwoBallTrajectoryTwo).deadlineWith(
        new IntakeAndHopperAction(),
        new TurretAction()
        ),
      new WaitCommand(1),
      new ShootAction().withTimeout(1),

      new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.CLOSE)),
      new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.OFF)),
      new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.OFF))
    );
  }
}
