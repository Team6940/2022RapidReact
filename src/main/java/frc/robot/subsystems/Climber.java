// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX m_leftclimberMastermotor;
  WPI_TalonFX m_rghtclimberFollowermotor;
  Solenoid m_climbersolenoid;
  private static Climber instance = null;
  PeriodicIO periodicIO = new PeriodicIO();
  private ClimberControlState currentState = ClimberControlState.Climber_Nothing;
  private int num_ElasticClimber = 1;
  private int num_StraightClimber = 1;

  public Climber() {
    m_leftclimberMastermotor = new WPI_TalonFX(Constants.leftClimberMotorPort);
    m_rghtclimberFollowermotor = new WPI_TalonFX(Constants.rghtClimberMotorPort);

    m_rghtclimberFollowermotor.setInverted(true);//TODO
    m_rghtclimberFollowermotor.follow(m_leftclimberMastermotor);//TODO

    m_climbersolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberSolenoidPort);
  }

  public static Climber getInstance() {
    if (instance == null){
        instance = new Climber();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(currentState == ClimberControlState.Climber_Nothing){
      m_leftclimberMastermotor.set(ControlMode.PercentOutput, 0);
    }else if(currentState == ClimberControlState.ElasticClimber_Forward){
      m_leftclimberMastermotor.set(ControlMode.PercentOutput, Constants.ClimberForwardSpeed);
    }else if(currentState == ClimberControlState.ElasticClimber_Reverse){
      m_leftclimberMastermotor.set(ControlMode.PercentOutput, Constants.ClimberBackwardSpeed);
    }else if(currentState == ClimberControlState.StraightClimber_On){
      m_climbersolenoid.set(true);
    }else if(currentState == ClimberControlState.StraightClimber_Off){
      m_climbersolenoid.set(false);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Climer Speed", m_leftclimberMastermotor.getMotorOutputPercent());
  }

  public void autosetElasticClimber(){
    if(num_ElasticClimber % 2 == 1){
      currentState = ClimberControlState.ElasticClimber_Forward;
    }else{
      currentState = ClimberControlState.ElasticClimber_Reverse;
    }
    num_ElasticClimber += 1;
  }

  public void stopElasticClimber(){
    currentState = ClimberControlState.Climber_Nothing;
  }

  public void autosetStraighClimber(){
    if(num_StraightClimber % 2 == 1){
      currentState = ClimberControlState.StraightClimber_On;
    }else{
      currentState = ClimberControlState.StraightClimber_Off;
    }
    num_StraightClimber += 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ClimberControlState {
    Climber_Nothing,ElasticClimber_Forward,ElasticClimber_Reverse,StraightClimber_On,StraightClimber_Off
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
