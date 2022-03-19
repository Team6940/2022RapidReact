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

public class Feeder extends SubsystemBase {
  /** Creates a new Intaker. */
  WPI_TalonFX m_intakermotor;
  WPI_TalonFX m_ballloadermotor;
  WPI_TalonFX m_blockermotor;
  Solenoid m_intakesolenoid;
  PeriodicIO periodicIO = new PeriodicIO();
  private FeederControlState currentState = FeederControlState.IntakeAndBallLoader_Off;
  private static Feeder instance = null;
  private int num = 1;

  public Feeder() {
    m_intakermotor = new WPI_TalonFX(Constants.IntakerPort);
    m_intakesolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakerSolenoidPort);

    m_ballloadermotor = new WPI_TalonFX(Constants.BallLoaderPort);

    m_blockermotor = new WPI_TalonFX(Constants.BlockerMotorPort);
  }

  public static Feeder getInstance() {
    if (instance == null){
        instance = new Feeder();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(currentState == FeederControlState.IntakeAndBallLoader_On){
      m_intakermotor.set(ControlMode.PercentOutput, 1);
      m_ballloadermotor.set(ControlMode.PercentOutput, Constants.BallLoadSpeed);
      setSolenoidState(true);
    }else if(currentState == FeederControlState.IntakeAndBallLoader_Off){
      m_intakermotor.set(ControlMode.PercentOutput, 0);
      m_ballloadermotor.set(ControlMode.PercentOutput, 0);
      setSolenoidState(false);
    }else if(currentState == FeederControlState.OnlyBallLoader_On){
      m_ballloadermotor.set(ControlMode.PercentOutput, Constants.BallLoadSpeed);
    }else if(currentState == FeederControlState.OnlyBallLoader_Off){
      m_ballloadermotor.set(ControlMode.PercentOutput, 0);
    }else if(currentState == FeederControlState.BallLocker_On){
      m_blockermotor.set(ControlMode.PercentOutput, Constants.BlockerMotorSpeed);
    }else if(currentState == FeederControlState.BallLocker_Off){
      m_blockermotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Intake output", m_intakermotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Ball Loader output", m_ballloadermotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Blocker Speed", m_blockermotor.getMotorOutputPercent());
  }

  public void autoturnintaker(){
    if(num % 2 == 1){
      currentState = FeederControlState.IntakeAndBallLoader_On;
    }
    else{
      currentState = FeederControlState.IntakeAndBallLoader_Off;
    }
    num += 1;
  }

  public void setSolenoidState(boolean status){
    m_intakesolenoid.set(status);
  }

  public void turnonballLoader(){
    currentState = FeederControlState.OnlyBallLoader_On;
  }

  public void turnoffballLoader(){
    currentState = FeederControlState.OnlyBallLoader_Off;
  }

  public void turnonballLocker(){
    currentState = FeederControlState.BallLocker_On;
  }

  public void turnoffballLocker(){
    currentState = FeederControlState.BallLocker_Off;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum FeederControlState {
    IntakeAndBallLoader_On,IntakeAndBallLoader_Off,OnlyBallLoader_On,OnlyBallLoader_Off,BallLocker_On,BallLocker_Off
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}