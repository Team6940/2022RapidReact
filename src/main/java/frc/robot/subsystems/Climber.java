// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX m_leftclimbermotor;
  WPI_TalonFX m_rghtclimbermotor;
  Solenoid m_climbersolenoid;
  private static Climber instance = null;

  public Climber() {
    m_leftclimbermotor = new WPI_TalonFX(Constants.leftClimberMotorPort);
    m_rghtclimbermotor = new WPI_TalonFX(Constants.rghtClimberMotorPort);
    m_climbersolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberSolenoidPort);
  }

  public static Climber getInstance() {
    if (instance == null){
        instance = new Climber();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
