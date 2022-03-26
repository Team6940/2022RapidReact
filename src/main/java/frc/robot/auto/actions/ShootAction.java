// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blocker;

public class ShootAction extends CommandBase {
  /** Creates a new ShootAction. */
  Blocker vBlocker = Blocker.getInstance();
  public ShootAction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vBlocker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vBlocker.turnonballLocker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vBlocker.turnoffballLocker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
