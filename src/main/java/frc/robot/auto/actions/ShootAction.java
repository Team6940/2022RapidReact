// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class ShootAction extends CommandBase {
  /** Creates a new ShootAction. */
  Feeder mFeeder = Feeder.getInstance();
  public ShootAction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFeeder.turnonballLocker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFeeder.turnoffballLocker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
