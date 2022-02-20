// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.lib.AutoRel.PPSwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerFollowTrajectory extends SequentialCommandGroup {
  /**
   * This constrcuts a PathPlanner Trajectory with swerve head control
   * @param m_swerve Your swerve
   * @param trajectoryJSON  The name of your trajectory
   * @param maxSpeed  The max speed of the path
   * @param maxAccel The max acceleration speed of the path
   */
  public PathPlannerFollowTrajectory(
    SwerveDriveTrain m_swerve,
    String trajectoryJSON,
    double maxSpeed,
    double maxAccel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(trajectoryJSON, maxSpeed, maxAccel);

    var thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //thetaController.setTolerance(0.1,0.1);

    PPSwerveControllerCommand ppSwerveControllerCommand = 
      new PPSwerveControllerCommand(
        examplePath,
        m_swerve::GetPose, // Functional interface to feed supplier
        SwerveDriveTrain.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_swerve::SetModuleStates,
        m_swerve);

    addCommands( 
        /**Reset the odometry to thte start pose of the trajectory */
        new InstantCommand(() -> m_swerve.ResetOdometry(examplePath.getInitialPose())),

        /**Reset the profiled PID controller */ 
        new InstantCommand(() -> thetaController.reset(m_swerve.GetPose().getRotation().getRadians())),

        ppSwerveControllerCommand
    );
  }
}
