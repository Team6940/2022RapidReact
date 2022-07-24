// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixyCamSPI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //LedSubsystem.getInstance().writePeriodicOutputs();

    Feeder.getInstance().writePeriodicOutputs();
    Feeder.getInstance().outputTelemetry();

    Blocker.getInstance().writePeriodicOutputs();
    Blocker.getInstance().outputTelemetry();

    //ClimberOld.getInstance().writePeriodicOutputs();
    //ClimberOld.getInstance().outputTelemetry();

    //PixyCamSPI.getInstance().readPeriodicInputs();
    //PixyCamSPI.getInstance().outputTelemetry();
    //PixyCamSPI.getInstance().writePeriodicOutputs();
    Turret.getInstance().readPeriodicInputs();
    Shooter.getInstance().readPeriodicInputs();
    Hood.getInstance().readPeriodicInputs();
    if(LimelightSubsystem.getInstance().getLightMode() == 3){
      Turret.getInstance().writePeriodicOutputs();
      Shooter.getInstance().writePeriodicOutputs();
    }
    Turret.getInstance().outputTelemetry();
    Shooter.getInstance().outputTelemetry();
    Hood.getInstance().outputTelemetry();

    //Shooter.getInstance().readHoodAngleFromShuffleBoard();
    //Shooter.getInstance().readShooterSpeedFromShuffleBoard();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    ColorSensor.getInstance().getTargetBallUpdate();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      //m_robotContainer.m_swerve.ResetOdometry(m_robotContainer.);
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.m_swerve.ZeroHeading();
    RobotContainer.m_swerve.whetherstoreyaw = false;
    ColorSensor.getInstance().getTargetBallUpdate();
    Turret.getInstance().ZeroTurret();
    Hood.getInstance().ZeroHood();
    Shooter.getInstance().setStopShooter();
    LimelightSubsystem.getInstance().reloadLimeLightSimu();
    Turret.getInstance().startVisionFinding();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(LimelightSubsystem.getInstance().getLightMode() == 3){
      if(Turret.getInstance().isStop()){
        Turret.getInstance().startVisionFinding();
      }
    }else{
      Turret.getInstance().Stop();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
