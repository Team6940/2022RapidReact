// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.team1706.MathUtils;
import frc.robot.lib.team2910.control.PidConstants;
import frc.robot.lib.team2910.control.PidController;
import frc.robot.subsystems.AimManager;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Hopper.HopperState;

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
  final double ANGULAR_D = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private static final PidConstants PID_CONSTANTS = new PidConstants(0.5, 0, 0);
  private static final double ROTATION_STATIC_CONSTANT = 0.3;
  private static final double MAXIMUM_AVERAGE_VELOCITY = 2.0;
  
  private PidController controller = new PidController(PID_CONSTANTS);
  private double lastTime = 0.0;

  ProfiledPIDController thetaController =
  new ProfiledPIDController(
      0.3,
      0,
      0,
      AutoConstants.kThetaAimControllerConstraints);
      
    
  Translation2d translation;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);
    
  public AutoAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
    addRequirements(RobotContainer.m_limelight);
    addRequirements(RobotContainer.m_aimManager);
    addRequirements(RobotContainer.m_shooter);

    controller.setInputRange(-Math.PI, Math.PI);
    controller.setContinuous(true);
    controller.setIntegralRange(Math.toRadians(10.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.reset(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_swerve.auto = true;
    RobotContainer.m_limelight.setLightMode(3);
    lastTime = Timer.getFPGATimestamp();
    controller.reset();
    //thetaController.reset(measurement);//TODO
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    double dt = time - lastTime;
    lastTime = time;

    double forwardSpeed;
    double rotationSpeed;

    double totalforwardSpeed;
    double totalrotationSpeed;

    double translationX = -inputTransform(RobotContainer.m_driverController.getLeftY());
    double translationY = -inputTransform(RobotContainer.m_driverController.getLeftX());
    //double rotationNew = -inputTransform(RobotContainer.m_driverController.getRightX());
    double lastz = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getRightX());
    double llastz = lastz * 5;

    Translation2d translation = new Translation2d(m_slewX.calculate(
      translationX) * SwerveConstants.kMaxSpeed,
        m_slewY.calculate(translationY) * SwerveConstants.kMaxSpeed);

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
        rotationSpeed = -1.0 * turnController.calculate(
            RobotContainer.m_limelight.Get_tx() /*+ AimManager.getInstance().getTxOffset()*/,
            0);
      } else {
          // If we have no targets, stay still.
          forwardSpeed = 0;
          rotationSpeed = 0;
        } 

      totalforwardSpeed = /*forwardSpeed * Constants.kMaxSpeed*/ + translationX;
      totalrotationSpeed = rotationSpeed * 5 + llastz;

      // Use our forward/turn speeds to control the drivetrain
      //RobotContainer.m_swerve.Drive(forwardSpeed, 0, rotationSpeed, false);

      double rotationalVelocity = 0.0;
      if (LimelightSubsystem.getInstance().isTargetVisible()) {
        //double currentAngle = drivetrain.getPose().rotation.toRadians();
        //double targetAngle = visionSubsystem.getAngleToTarget().getAsDouble();
        double targetTx = RobotContainer.m_limelight.Get_tx();
        controller.setSetpoint(0);
        rotationalVelocity = controller.calculate(targetTx, dt);

        //if (drivetrain.getAverageAbsoluteValueVelocity() < MAXIMUM_AVERAGE_VELOCITY) {
        //    rotationalVelocity += Math.copySign(
        //            ROTATION_STATIC_CONSTANT / RobotController.getBatteryVoltage(),
        //            rotationalVelocity
        //    );
        //}
      }
      
      double rotationSpeed2 = thetaController.calculate(RobotContainer.m_limelight.Get_tx(), 0);

      RobotContainer.m_swerve.Drive(translation, -totalrotationSpeed, true, true);
      
      // Goal-Centric
      //RobotContainer.m_swerve.Drive(
      //    translation,
      //    -totalrotationSpeed * DriveConstants.kMaxAngularSpeed, //If it does work well, remove the rotation slew rate limiter
      //    true,
      //    true);//Use feedback control when auto aiming.
      //RobotContainer.m_swerve.Drive(translation, -rotationalVelocity, true, true);//Use feedback control when auto aiming.
      //RobotContainer.m_swerve.Drive(translation, - rotationSpeed2, true, true);//Use feedback control when auto aiming.

      RobotContainer.m_aimManager.startAimShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.m_limelight.setLightMode(1);
    RobotContainer.m_aimManager.Stop();
    //RobotContainer.m_shooter.setFiring(false);
    RobotContainer.m_shooter.setShooterToStop();
    RobotContainer.m_hopper.setHopperState(HopperState.OFF);
    RobotContainer.m_swerve.Drive(new Translation2d(0,0), 0, true, true);//TODO
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    /**
   * This function takes the user input from the controller analog sticks, applys
   * a deadband and then quadratically
   * transforms the input so that it is easier for the user to drive, this is
   * especially important on high torque motors
   * such as the NEOs or Falcons as it makes it more intuitive and easier to make
   * small corrections
   * 
   * @param input is the input value from the controller axis, should be a value
   *              between -1.0 and 1.0
   * @return the transformed input value
   */
  private double inputTransform(double input) {
    return MathUtils.signedSquare(MathUtils.applyDeadband(input));
  }
}
