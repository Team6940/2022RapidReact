// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blocker extends SubsystemBase {
  /** Creates a new Blocker. */
  WPI_TalonFX m_blockermotor;

  public Blocker() {
    m_blockermotor = new WPI_TalonFX(Constants.BlockerMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
