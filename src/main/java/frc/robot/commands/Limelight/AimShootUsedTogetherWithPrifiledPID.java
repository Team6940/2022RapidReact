// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
When running this command,the shooter will spin at the proper speed when having locked the target according to the distance 
This Command is added to solve the problem that we can't do the shooting in the ProfiledPID Command used for moving the basement  
The command will be run together with the AutoAimUsingProfiledPID command 

*/
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
import frc.robot.Robot;
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

public class AimShootUsedTogetherWithPrifiledPID extends CommandBase{
    public AimShootUsedTogetherWithPrifiledPID()
    {
        addRequirements(RobotContainer.m_limelight);
        addRequirements(RobotContainer.m_aimManager);
        addRequirements(RobotContainer.m_shooter);
    }
    @Override
     public void initialize() {
    RobotContainer.m_swerve.auto = true;
    RobotContainer.m_limelight.setLightMode(3);
    //thetaController.reset(measurement);//TODO
  }
    @Override
    public void execute() {
        RobotContainer.m_aimManager.startAimShoot();
    }
    @Override
    public void end(boolean interrupted) {
        //RobotContainer.m_limelight.setLightMode(1);
        RobotContainer.m_aimManager.Stop();
        //RobotContainer.m_shooter.setFiring(false);
        RobotContainer.m_shooter.setShooterToStop();
        //RobotContainer.m_hopper.setHopperState(HopperState.OFF);
        RobotContainer.m_swerve.Drive(new Translation2d(0,0), 0, true, true);//TODO
    }
    @Override
    public boolean isFinished() {
      return false;
    }
}