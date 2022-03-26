// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootAction extends CommandBase {
  /** Creates a new ShootAction. */
  Feeder  vFeeder = Feeder.getInstance();
  Shooter vShooter = Shooter.getInstance();

  public ShootAction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vShooter.shootIsReady()){
      vFeeder.turnonballLocker();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vFeeder.turnoffballLocker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
