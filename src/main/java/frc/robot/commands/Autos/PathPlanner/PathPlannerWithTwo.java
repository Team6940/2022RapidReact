// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.PathPlanner;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.lib.AutoRel.SwerveControllerCommand;
import frc.robot.lib.AutoRel.PPSwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerWithTwo extends SequentialCommandGroup {
  /** Creates a new PathPlannerWithTwo. */

  private PathPlannerState getInitialState(PathPlannerTrajectory traj){
    return traj.getInitialState();
  }

  public PathPlannerWithTwo(SwerveDriveTrain m_swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory Path1 = PathPlanner.loadPath("Test Path 1", 4, 6);

    PathPlannerTrajectory Path2 = PathPlanner.loadPath("Test Path 2", 1, 1);

    Trajectory traj = openTrajectoryFromJSON("Test Path 1.wpilib.json");

    var thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController,
          AutoConstants.kIThetaController,
          AutoConstants.kDThetaController,
          AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //thetaController.setTolerance(0.1,0.1);

    SmartDashboard.putData("thetaController", thetaController);
    SmartDashboard.putNumber("current rotation", m_swerve.GetPose().getRotation().getRadians());

    PPSwerveControllerCommand path1command = 
      new PPSwerveControllerCommand(
        Path1,
        m_swerve::GetPose, // Functional interface to feed supplier
        SwerveDriveTrain.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::SetModuleStates,
        m_swerve);

    PPSwerveControllerCommand path2command = 
      new PPSwerveControllerCommand(
        Path2,
        m_swerve::GetPose, // Functional interface to feed supplier
        SwerveDriveTrain.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::SetModuleStates,
        m_swerve);

    SwerveControllerCommand WPI_path1command =
      new SwerveControllerCommand(
        traj,
        m_swerve::GetPose, // Functional interface to feed supplier
        SwerveDriveTrain.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::SetModuleStates,
        m_swerve);

    addCommands(
      new InstantCommand(() -> m_swerve.ResetOdometry(
        new Pose2d(
          Path1.getInitialState().poseMeters.getTranslation(),
          Path1.getInitialState().holonomicRotation
          )
          //traj.getInitialPose()
        )
      ),

      /**Reset the profiled PID controller */ 
      new InstantCommand(() -> thetaController.reset(m_swerve.GetPose().getRotation().getRadians())),

      //WPI_path1command
      path1command//,
      //path2command
    );
  }
        /**
     * Read the JSON output from pathweaver and convert it to a trajectory object
     *
     * @param JSONName the name of the JSON stored in the deploy/output directory, e.x. "bounceLeg1.wpilib.json"
     */
    private Trajectory openTrajectoryFromJSON (String JSONName) {
      JSONName = "pathplanner/generatedJSON/" + JSONName;

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
