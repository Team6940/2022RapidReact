// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallLoader extends SubsystemBase {
  /** Creates a new BallLoader. */
  WPI_TalonFX m_ballLoadermotor;
  PeriodicIO periodicIO = new PeriodicIO();
  private BallLoaderControlState currentstate = BallLoaderControlState.BallLoader_Off;
  private static BallLoader instance = null;
  private boolean whetherballloaderon = false;

  public BallLoader() {
    m_ballLoadermotor = new WPI_TalonFX(Constants.BallLoaderPort);
  }

  public static BallLoader getInstance() {
    if (instance == null){
        instance = new BallLoader();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(whetherballloaderon){
      currentstate = BallLoaderControlState.BallLoader_On;
    }
    else{
      currentstate = BallLoaderControlState.BallLoader_Off;
    }
    if(currentstate == BallLoaderControlState.BallLoader_On){
      m_ballLoadermotor.set(ControlMode.PercentOutput, 1);
    }
    else if(currentstate == BallLoaderControlState.BallLoader_Off){
      m_ballLoadermotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("BallLoader output", m_ballLoadermotor.getMotorOutputPercent());
  }

  public void turnonballloader(){
    whetherballloaderon = true;
  }

  public void turnoffballloader(){
    whetherballloaderon = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public enum BallLoaderControlState {
    BallLoader_On,BallLoader_Off
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
