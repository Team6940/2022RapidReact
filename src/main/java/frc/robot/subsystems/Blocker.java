// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blocker extends SubsystemBase {
  /** Creates a new Blocker. */
  WPI_TalonFX m_blockermotor;
  PeriodicIO periodicIO = new PeriodicIO();
  private BlockerControlState currentState = BlockerControlState.BALLLOCKER_OFF;
  private static Blocker instance = null;

  public Blocker() {
    m_blockermotor = new WPI_TalonFX(Constants.BlockerMotorPort);//设定blocker电机

    m_blockermotor.configVoltageCompSaturation(12);
    m_blockermotor.enableVoltageCompensation(true);//???
  }

  public static Blocker getInstance() {//创建一个Blocker
    if (instance == null){
        instance = new Blocker();
    }
    return instance;
  }

  public void writePeriodicOutputs(){//状态机输出
    if(currentState == BlockerControlState.BALLLOCKER_ON){//如果blocker处于On状态，让blocker电机定速旋转
      m_blockermotor.set(ControlMode.PercentOutput, Constants.BlockerMotorSpeed);
      //currentState = BlockerControlState.BALLLOCKER_OFF;
    }else if(currentState == BlockerControlState.BALLLOCKER_OFF){//如果blocker处于off状态，让blocker停止
      m_blockermotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Debug/Blocker/Speed", m_blockermotor.getMotorOutputPercent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoTurnOnBlocker();
  }

  public void turnonballLocker(){//打开blocker
    currentState = BlockerControlState.BALLLOCKER_ON;
  }

  public void turnoffballLocker(){//关闭blocker
    currentState = BlockerControlState.BALLLOCKER_OFF;
  }

  public void autoTurnOnBlocker(){//自动处理blocker开关
    if(VisionManager.getInstance().isShooterCanShoot()){//如果shooter可以射球，把blocker打开
      currentState = BlockerControlState.BALLLOCKER_ON;
    }else{
      currentState = BlockerControlState.BALLLOCKER_OFF;
    }
  }

  public enum BlockerControlState {//blocker的两种状态
    BALLLOCKER_ON,//blocker打开
    BALLLOCKER_OFF//blocker关闭
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }

  public BlockerControlState getBlockerState(){//获取blocker的当前状态
      return currentState;
  }
}
