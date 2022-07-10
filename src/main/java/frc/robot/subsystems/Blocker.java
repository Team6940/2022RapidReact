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
    m_blockermotor = new WPI_TalonFX(Constants.BlockerMotorPort);

    m_blockermotor.configVoltageCompSaturation(12);
    m_blockermotor.enableVoltageCompensation(true);
  }

  public static Blocker getInstance() {
    if (instance == null){
        instance = new Blocker();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(currentState == BlockerControlState.BALLLOCKER_ON){
      m_blockermotor.set(ControlMode.PercentOutput, Constants.BlockerMotorSpeed);
      //currentState = BlockerControlState.BALLLOCKER_OFF;
    }else if(currentState == BlockerControlState.BALLLOCKER_OFF){
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

  public void turnonballLocker(){
    currentState = BlockerControlState.BALLLOCKER_ON;
  }

  public void turnoffballLocker(){
    currentState = BlockerControlState.BALLLOCKER_OFF;
  }

  public void autoTurnOnBlocker(){
    if(Shooter.getInstance().whetherReadyToShoot()){
      currentState = BlockerControlState.BALLLOCKER_ON;
    }else{
      currentState = BlockerControlState.BALLLOCKER_OFF;
    }
  }

  public enum BlockerControlState {
    BALLLOCKER_ON,
    BALLLOCKER_OFF
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
