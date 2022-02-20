// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends SequentialCommandGroup {
  /** Creates a new FollowTrajectory. */
  private Trajectory trajectory;

  public FollowTrajectory(SwerveDriveTrain m_swerve, String trajectoryJSON) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*TrajectoryConfig config =
        new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveTrain.kDriveKinematics);*/

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(0.5, 0.5), new Translation2d(1, -0.5)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(2, 0, new Rotation2d(0)),
          Constants.AutoConstants.defaultConfig);

    trajectory = openTrajectoryFromJSON(trajectoryJSON); //Load the pathweaver trajectory

    var thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
          //trajectory,
          exampleTrajectory,
          m_swerve::GetPose, // Functional interface to feed supplier
          SwerveDriveTrain.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_swerve::SetModuleStates,
          m_swerve);

    addCommands( 
        new InstantCommand(() -> m_swerve.ResetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand);
   }

      /**
     * Read the JSON output from pathweaver and convert it to a trajectory object
     *
     * @param JSONName the name of the JSON stored in the deploy/output directory, e.x. "bounceLeg1.wpilib.json"
     */
    private Trajectory openTrajectoryFromJSON (String JSONName) {
      JSONName = "output/" + JSONName;

      Trajectory trajectory = new Trajectory();
      try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONName);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          DriverStation.reportError("Couldn't load trajectory", true);
      }

      return trajectory;
  }
}
