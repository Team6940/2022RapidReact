// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intaker extends SubsystemBase {
  /** Creates a new Intaker. */
  WPI_TalonFX m_intakermotor;
  PeriodicIO periodicIO = new PeriodicIO();
  private IntakerControlState currentState = IntakerControlState.Intake_Off;
  private static Intaker instance = null;
  private boolean whetherintakeron = false;

  public Intaker() {
    m_intakermotor = new WPI_TalonFX(Constants.IntakerPort);
  }

  public static Intaker getInstance() {
    if (instance == null){
        instance = new Intaker();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(whetherintakeron){
      currentState = IntakerControlState.Intake_On;
    }
    else{
      currentState = IntakerControlState.Intake_Off;
    }
    if(currentState == IntakerControlState.Intake_On){
      m_intakermotor.set(ControlMode.PercentOutput, 1);
    }
    else if(currentState == IntakerControlState.Intake_Off){
      m_intakermotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Intake output", m_intakermotor.getMotorOutputPercent());
  }

  public void turnonintaker(){
    whetherintakeron = true;
  }

  public void turnoffintaker(){
    whetherintakeron = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum IntakerControlState {
    Intake_On,Intake_Off
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
