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
import frc.robot.auto.actions.ShootAction;
import frc.robot.auto.actions.SwervePathAction;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeSolState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallBottomMode2 extends SequentialCommandGroup {
  /** Creates a new FiveBallBottomMode. */
  public FiveBallBottomMode2(SwerveDriveTrain sSwerve) {
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
      
      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mFiveBallTrajectoryOne).deadlineWith(
           new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.OPEN)),
           new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.INTAKE)),
           new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.ON)),
           new InstantCommand(() -> Turret.getInstance().startVisionFinding())),
      new WaitCommand(0.5),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mFiveBallTrajectoryTwo).deadlineWith(
        new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.OPEN)),
        new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.INTAKE)),
        new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.ON)),
        new InstantCommand(() -> Turret.getInstance().startVisionFinding())),
      new ShootAction().withTimeout(1),
      //new WaitCommand(0.5),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mFiveBallTrajectoryThree).deadlineWith(
        new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.OPEN)),
        new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.INTAKE)),
        new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.ON)),
        new InstantCommand(() -> Turret.getInstance().startVisionFinding())),
      new WaitCommand(0.5),
      new ShootAction().withTimeout(1),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mFiveBallTrajectoryFour).deadlineWith(
        new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.OPEN)),
        new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.INTAKE)),
        new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.ON)),
        new InstantCommand(() -> Turret.getInstance().startVisionFinding())),
      new WaitCommand(2),
      //new WaitCommand(0.5),

      new InstantCommand(() -> LimelightSubsystem.getInstance().reloadLimeLightSimu()),
      new SwervePathAction(mFiveBallTrajectoryFive).deadlineWith(
        new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.OPEN)),
        new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.INTAKE)),
        new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.ON)),
        new InstantCommand(() -> Turret.getInstance().startVisionFinding())),
      new ShootAction().withTimeout(1),

      new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.CLOSE)),
      new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.OFF)),
      new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.OFF))
    );
  }
}
