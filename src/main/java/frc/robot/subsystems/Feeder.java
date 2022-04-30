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
  Solenoid m_intakesolenoid;
  PeriodicIO periodicIO = new PeriodicIO();
  private FeederControlState currentState = FeederControlState.INATKEANDBALLLOADER_OFF;
  private static Feeder instance = null;
  private int num = 1;

  public Feeder() {
    m_intakermotor = new WPI_TalonFX(Constants.IntakerPort);
    m_intakesolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakerSolenoidPort);

    m_ballloadermotor = new WPI_TalonFX(Constants.BallLoaderPort);

    m_intakermotor.configVoltageCompSaturation(12);
    m_intakermotor.enableVoltageCompensation(true);

    m_ballloadermotor.configVoltageCompSaturation(12);
    m_ballloadermotor.enableVoltageCompensation(true);
  }

  public static Feeder getInstance() {
    if (instance == null){
        instance = new Feeder();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(currentState == FeederControlState.INTAKEANDBALLLOADER_ON){
      m_intakermotor.set(ControlMode.PercentOutput, 1);
      m_ballloadermotor.set(ControlMode.PercentOutput, Constants.BallLoadSpeed);
      setSolenoidState(true);
    }else if(currentState == FeederControlState.INATKEANDBALLLOADER_OFF){
      m_intakermotor.set(ControlMode.PercentOutput, 0);
      m_ballloadermotor.set(ControlMode.PercentOutput, 0);
      setSolenoidState(false);
    }else if(currentState == FeederControlState.ONLYBALLLOADER_ON){
      m_ballloadermotor.set(ControlMode.PercentOutput, Constants.BallLoadSpeed);
    }else if(currentState == FeederControlState.ONLYBALLLOADER_OFF){
      m_ballloadermotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Intake output", m_intakermotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Ball Loader output", m_ballloadermotor.getMotorOutputPercent());
  }

  public void autoturnintaker(){
    if(num % 2 == 1){
      currentState = FeederControlState.INTAKEANDBALLLOADER_ON;
    }
    else{
      currentState = FeederControlState.INATKEANDBALLLOADER_OFF;
    }
    num += 1;
  }

  public void setSolenoidState(boolean status){
    m_intakesolenoid.set(status);
  }

  public void turnonballLoader(){
    currentState = FeederControlState.ONLYBALLLOADER_ON;
  }

  public void turnoffballLoader(){
    currentState = FeederControlState.ONLYBALLLOADER_OFF;
  }

  public void setIntakeandBallLoaderOn(){
    currentState = FeederControlState.INTAKEANDBALLLOADER_ON;
  }

  public void setIntakeandBallLoaderOff(){
    currentState = FeederControlState.INATKEANDBALLLOADER_OFF;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum FeederControlState {
    INTAKEANDBALLLOADER_ON,
    INATKEANDBALLLOADER_OFF,
    ONLYBALLLOADER_ON,
    ONLYBALLLOADER_OFF,
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
