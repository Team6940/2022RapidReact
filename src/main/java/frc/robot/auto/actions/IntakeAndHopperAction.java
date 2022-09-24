// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeSolState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LimelightSubsystem;

public class IntakeAndHopperAction extends CommandBase {
  /** Creates a new IntakeBall. */
  Intake mIntake = Intake.getInstance();
  Hopper mHopper = Hopper.getInstance();

  public boolean SwitchIntake = true;

  public IntakeAndHopperAction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntake,mHopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.setIntakeSolState(IntakeSolState.OPEN);
    mIntake.setWantedIntakeState(IntakeState.INTAKE);
    mHopper.setHopperState(HopperState.ON);
    LimelightSubsystem.getInstance().reloadLimeLightSimu();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mHopper.setHopperState(HopperState.OFF);
    mIntake.setWantedIntakeState(IntakeState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
