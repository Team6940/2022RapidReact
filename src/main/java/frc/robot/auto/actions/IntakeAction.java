// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LimelightSubsystem;

public class IntakeAction extends CommandBase {
  /** Creates a new IntakeBall. */
  Feeder mFeeder = Feeder.getInstance();

  public boolean SwitchIntake = true;

  public IntakeAction(boolean vSwitchintake) {
    // Use addRequirements() here to declare subsystem dependencies.
    SwitchIntake = vSwitchintake;
    addRequirements(mFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFeeder.setIntakeandBallLoaderOn();
    LimelightSubsystem.getInstance().reloadLimeLightSimu();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(SwitchIntake){
      mFeeder.setIntakeandBallLoaderOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
