// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AimManager;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20 / 2.54);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0) + Units.inchesToMeters(55/2.54);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);
        
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(0.5/0.3048);
  
  // PID constants should be tuned per robot
  final double LINEAR_P = 0.01;
  final double LINEAR_D = 0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    
  final double ANGULAR_P = 0.02;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  Translation2d translation;
    
  public AutoAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
    addRequirements(RobotContainer.m_limelight);
    addRequirements(RobotContainer.m_aimManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_swerve.auto = true;
    RobotContainer.m_limelight.setLightMode(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;

    double totalforwardSpeed;
    double totalrotationSpeed;

    double lastx = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftY());
    double lasty = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftX());
    double lastz = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getRightX());

    double llastx = - RobotContainer.m_swerve.deadband(lastx) * Constants.kMaxSpeed;
    double llasty = - RobotContainer.m_swerve.deadband(lasty) * Constants.kMaxSpeed;
    double llastz = lastz * Constants.kMaxOmega;

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    double target  = RobotContainer.m_limelight.Get_tv();

    if (target != 0.0f) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(RobotContainer.m_limelight.Get_ty()));
        //SmartDashboard.putNumber("Debug/Auto/range", range);
        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        if(RobotContainer.m_limelight.Get_ty()<-1){
          forwardSpeed = 1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);;
        }
        else if(RobotContainer.m_limelight.Get_ty() > 1){
          forwardSpeed = -1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);
        }
        else{
          forwardSpeed = 0;
        }


        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -1.0 * turnController.calculate(RobotContainer.m_limelight.Get_tx(), 0);
      } else {
          // If we have no targets, stay still.
          forwardSpeed = 0;
          rotationSpeed = 0;
        } 

      totalforwardSpeed = /*forwardSpeed * Constants.kMaxSpeed*/ + llastx;
      totalrotationSpeed = rotationSpeed * Constants.kMaxOmega + llastz;

      translation = new Translation2d(totalforwardSpeed, llasty);

      // Use our forward/turn speeds to control the drivetrain
      //RobotContainer.m_swerve.Drive(forwardSpeed, 0, rotationSpeed, false);

      // Goal-Centric
      RobotContainer.m_swerve.Drive(translation, - totalrotationSpeed, true, false);//Use feedback control when auto aiming.

      RobotContainer.m_aimManager.startAimShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_limelight.setLightMode(1);
    RobotContainer.m_aimManager.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
