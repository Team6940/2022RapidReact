// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TurretAndShooterAction extends CommandBase {
  /** Creates a new TurretAction. */
  Turret mTurret = Turret.getInstance();
  Shooter mShooter = Shooter.getInstance();

  public TurretAndShooterAction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTurret,mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTurret.startVisionFinding();
    mShooter.setPrepareShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
