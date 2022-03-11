// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControll extends CommandBase {
  /** Creates a new SwerveControll. */
  double storedYaw;
  
  private double rotation;
  private Translation2d translation;

  public SwerveControll() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
  }

  private double applyRotationalDeadband(double input){
    double deadband = Constants.stickDeadband;
    if (Math.abs(input) < deadband) {
        return 0.0;
    } else {
        return (input - (Math.signum(input) * deadband)) / (1 - deadband);
    }
  }

  private Translation2d applyTranslationalDeadband(Translation2d input) {
    double deadband = Constants.stickDeadband;
    if (Math.abs(input.getNorm()) < deadband) {
        return new Translation2d();
    } else {
        Rotation2d deadband_direction = new Rotation2d(input.getX(), input.getY());
        Translation2d deadband_vector = new Translation2d(deadband, deadband_direction);

        double scaled_x = input.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
        double scaled_y = input.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());
        return new Translation2d(scaled_x, scaled_y);
    }

  }

  public double[] getAxes() {
    double yAxis = -RobotContainer.m_driverController.getLeftY();
    double xAxis = -RobotContainer.m_driverController.getLeftX();
    double rAxis = -RobotContainer.m_driverController.getRightX();

    Translation2d tAxes;
    
    /* Deadbands */
    tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
    rAxis = applyRotationalDeadband(rAxis);

    double[] axes = {tAxes.getX(), tAxes.getY(), rAxis};

    return axes;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storedYaw = RobotContainer.m_swerve.GetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double llastx = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftY());
    double llasty = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftX());
    double llastz = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getRightX());

    double yaw = RobotContainer.m_swerve.GetYaw();
    double yawCorrection = 0;

    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("storedyaw", storedYaw);
    SmartDashboard.putNumber("llastz", llastz);
    SmartDashboard.putNumber("yawcorrection", yawCorrection);
    SmartDashboard.putBoolean("WhetherStroeYaw", RobotContainer.m_swerve.whetherstoreyaw);

    //Try to optimize the input
    double yAxis;
    double xAxis;
    double rAxis;

    Translation2d tAxes; // translational axis

    /* Inversions */
    yAxis = -RobotContainer.m_driverController.getLeftY();
    xAxis = -RobotContainer.m_driverController.getLeftX();
    rAxis = RobotContainer.m_driverController.getRightX();

    /* Deadbands */
    tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
    rAxis = applyRotationalDeadband(rAxis);

    translation = new Translation2d(tAxes.getX(), tAxes.getY()).times(Constants.kMaxSpeedinTeleop);
    rotation = rAxis * Constants.kMaxOmega;

    if(RobotContainer.m_swerve.whetherstoreyaw || llastz != 0){
      storedYaw = yaw;
    }else{
      if(Math.abs(tAxes.getX()) > 0|| Math.abs(tAxes.getY()) > 0){
        yawCorrection = RobotContainer.m_swerve.calcYawStraight(storedYaw, yaw, 0.006, 0);
      }
    }

    /*rotation = (rAxis + yawCorrection) * Constants.kMaxOmega;*/

    RobotContainer.m_swerve.Drive(
      translation,
      -  (llastz + yawCorrection) * Constants.kMaxOmega/*rotation + yawCorrection * Constants.kMaxOmega*/,
      true,
      RobotContainer.m_swerve.isOpenLoop);

    //SmartDashboard.putNumber("X Controller Input", translation.getX());
    //SmartDashboard.putNumber("Y Controller Input", translation.getY());
    //SmartDashboard.putNumber("Rot Controller Input", rotation);
    SmartDashboard.putNumber("storedYaw", storedYaw);
    SmartDashboard.putNumber("origin_yaw", yaw);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
