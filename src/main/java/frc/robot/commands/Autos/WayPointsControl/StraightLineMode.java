// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.WayPointsControl;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Autos.WaitToSpinUpCommand;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightLineMode extends SequentialCommandGroup {
  /** Creates a new StraightLineMode. */
  public StraightLineMode(SwerveDriveTrain s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Trajectory firstStraightLine =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
            List.of(new Translation2d(0.25,0),new Translation2d(0.5, 0)),
            new Pose2d(0.75, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.RTNfastConfig);

    Trajectory secondstrafeLine = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.75, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(0.75, 0.25), new Translation2d(0.75, 0.5)),
            new Pose2d(0.75, 1, Rotation2d.fromDegrees(0.0)),
            Constants.AutoConstants.slowConfig);

    //secondstrafeLine = secondstrafeLine.transformBy(
    //    new Transform2d(
    //        secondstrafeLine.getInitialPose(), 
    //        firstStraightLine.sample(firstStraightLine.getTotalTimeSeconds()).poseMeters)
    //);

    Trajectory scurveLine = 
        TrajectoryGenerator.generateTrajectory(            
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.76, -0.8), new Translation2d(2.2, -0.14)),
            new Pose2d(1.07, 0.8, new Rotation2d(0)),
            Constants.AutoConstants.defaultConfig);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand firstStraightLineCommand =
    new SwerveControllerCommand(
        firstStraightLine,
        s_Swerve::GetPose,
        Constants.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        () -> Rotation2d.fromDegrees(0), //Swerve Heading
        s_Swerve::SetModuleStates,
        s_Swerve);

    SwerveControllerCommand secondstrafeLineCommand =
    new SwerveControllerCommand(
        secondstrafeLine,
        s_Swerve::GetPose,
        Constants.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        () -> Rotation2d.fromDegrees(0), //Swerve Heading
        s_Swerve::SetModuleStates,
        s_Swerve);

    SwerveControllerCommand scurveLineCommand =
    new SwerveControllerCommand(
        scurveLine,
        s_Swerve::GetPose,
        Constants.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        () -> Rotation2d.fromDegrees(0), //Swerve Heading
        s_Swerve::SetModuleStates,
        s_Swerve);

    WaitToSpinUpCommand waitToSpinUp =
        new WaitToSpinUpCommand(0.5);

    addCommands(
      new InstantCommand(() -> s_Swerve.ResetOdometry(scurveLine.getInitialPose())),
      new SequentialCommandGroup(
          //scurveLineCommand
          //firstStraightLineCommand.deadlineWith(
          //    waitToSpinUp
          //),
          scurveLineCommand
      ),
      new InstantCommand(() -> s_Swerve.Drive(new Translation2d(0,0), 0, false, false))
    );
  }
}
