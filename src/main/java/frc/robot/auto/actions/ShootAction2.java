// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AimManager;

public class ShootAction2 extends CommandBase {
  /** Creates a new ShootAction. */
  Shooter shooter = Shooter.getInstance();
  AimManager aimManager = AimManager.getInstance();

  public ShootAction2() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,aimManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimManager.startAimMoving();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(aimManager.CanShot()){
      shooter.setFiring(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFiring(false);
    aimManager.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
