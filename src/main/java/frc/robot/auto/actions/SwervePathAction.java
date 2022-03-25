// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.PPSwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwervePathAction extends SequentialCommandGroup {
  /** Creates a new FiveBallBottom. */
  SwerveDriveTrain mSwerve = SwerveDriveTrain.getInstance();
  
  public SwervePathAction(PathPlannerTrajectory mTrajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    var thetaController =
    new ProfiledPIDController(
        AutoConstants.kPThetaController,
        AutoConstants.kIThetaController,
        AutoConstants.kDThetaController,
        AutoConstants.kThetaControllerConstraints);
     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand PathCommand = 
      new PPSwerveControllerCommand(
      mTrajectory,
      mSwerve::GetPose, // Functional interface to feed supplier
      SwerveDriveTrain.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      mSwerve::SetModuleStates,
      mSwerve);

    addCommands(
      /**Reset the profiled PID controller */ 
      new InstantCommand(() -> thetaController.reset(mSwerve.GetPose().getRotation().getRadians())),
      PathCommand
    );
 }
}
