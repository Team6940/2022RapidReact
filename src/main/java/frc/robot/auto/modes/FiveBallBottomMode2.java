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
import frc.robot.auto.actions.IntakeAndHopperAction;
import frc.robot.auto.actions.SwervePathAction;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeSolState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Limelight.AutoAim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallBottomMode2 extends SequentialCommandGroup {
  /** Creates a new FiveBallBottomMode. */
  public FiveBallBottomMode2(SwerveDriveTrain sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory mFiveBallTrajectoryOne = PathPlanner.loadPath("FiveBallBottom-1", 4, 8);
    PathPlannerTrajectory mFiveBallTrajectoryTwo = PathPlanner.loadPath("FiveBallBottom-2", 4, 8);
    PathPlannerTrajectory mFiveBallTrajectoryThree = PathPlanner.loadPath("FiveBallBottom-3", 4, 8);
    PathPlannerTrajectory mFiveBallTrajectoryFour = PathPlanner.loadPath("FiveBallBottom-4", 4, 8);
    PathPlannerTrajectory mFiveBallTrajectoryFive = PathPlanner.loadPath("FiveBallBottom-5", 4, 8);
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
      
      new SwervePathAction(mFiveBallTrajectoryOne).deadlineWith(
           new IntakeAndHopperAction()
           ),
      new WaitCommand(2),

      /*new SwervePathAction(mFiveBallTrajectoryTwo).deadlineWith(
        //new IntakeAndHopperAction()
        ),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(2),
      //new WaitCommand(0.5),

      new SwervePathAction(mFiveBallTrajectoryThree).deadlineWith(
        new IntakeAndHopperAction()
        ),
      new WaitCommand(2),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(2), //TODO redraw heading path

      new SwervePathAction(mFiveBallTrajectoryFour).deadlineWith(
        new InstantCommand(() -> Intake.getInstance().runIntaker())
        ),
      new WaitCommand(2),
      new InstantCommand(() -> Intake.getInstance().stopIntaker()), 
      //new WaitCommand(0.5),

      new SwervePathAction(mFiveBallTrajectoryFive).deadlineWith(
        ///new IntakeAndHopperAction()  
        ),
      //new ShootAction2().withTimeout(1),
      new AutoAim().withTimeout(3),*/

      new InstantCommand(() -> Intake.getInstance().setIntakeSolState(IntakeSolState.CLOSE)),
      new InstantCommand(() -> Intake.getInstance().setWantedIntakeState(IntakeState.OFF)),
      new InstantCommand(() -> Hopper.getInstance().setHopperState(HopperState.OFF))
    );
  }
}
