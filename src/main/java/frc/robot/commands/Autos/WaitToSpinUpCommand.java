// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitToSpinUpCommand extends CommandBase {
  /** Creates a new WaitToSpinUpCommand. */

  private double mTimeToWait;
  private double mStartTime;

  public WaitToSpinUpCommand(double waitTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTimeToWait = waitTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mStartTime = Timer.getFPGATimestamp();
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
    if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
      return true;
    }
    return false;
  }
}
