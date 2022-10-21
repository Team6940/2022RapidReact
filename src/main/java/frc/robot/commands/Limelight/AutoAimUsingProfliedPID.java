// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain;

/*

 */
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoAimUsingProfliedPID extends ProfiledPIDCommand {
    /** Creates a new RotateDrivetrainByLimelightAngle. */
    private boolean continuous;

    public AutoAimUsingProfliedPID(boolean continuous) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        AutoConstants.kPThetaController * .85,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        AutoConstants.kThetaAimControllerConstraints),
                // This should return the measurement
                () -> LimelightSubsystem.getInstance().Get_tx() * Math.PI / 180,
                // This should return the goal (can also be a constant)
                () -> 0.0,
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    SwerveModuleState[] states = SwerveConstants.swerveKinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    SwerveConstants.kMaxSpeed
                                            * RobotContainer.getInstance().getSpeedScaledDriverLeftY(),
                                    SwerveConstants.kMaxSpeed
                                            * RobotContainer.getInstance().getSpeedScaledDriverLeftX(),
                                    output + setpoint.velocity,
                                    SwerveDriveTrain.getInstance().getPose().getRotation()));
                    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeed);
                    SwerveDriveTrain.getInstance().SetModuleStates(states);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(SwerveDriveTrain.getInstance(), LimelightSubsystem.getInstance());
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(1.0));
        this.continuous = continuous;
    }

      // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {
    //     RobotContainer.m_aimManager.startAimShoot();
    // }

    // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {
    //     //RobotContainer.m_limelight.setLightMode(1);
    //     RobotContainer.m_aimManager.Stop();
    //     //RobotContainer.m_shooter.setFiring(false);
    //     RobotContainer.m_shooter.setShooterToStop();
    //     //RobotContainer.m_hopper.setHopperState(HopperState.OFF);
    //     RobotContainer.m_swerve.Drive(new Translation2d(0,0), 0, true, true);//TODO
    // }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!continuous) {
            return getController().atGoal();
        } else {
            return false;
        }
    }
}
