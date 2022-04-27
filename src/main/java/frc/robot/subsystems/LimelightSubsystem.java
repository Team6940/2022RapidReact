// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

public class LimelightSubsystem extends SubsystemBase {

  private static LimelightSubsystem instance = null;
  /** Creates a new Limelight. */
  public NetworkTable m_limTable;

  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;
  public double simTx = 30 ;
  public double simTv = 10.0 ;
  
  public LimelightSubsystem() {
    m_limTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public static LimelightSubsystem getInstance() {
    if (instance == null){
      instance = new LimelightSubsystem();
    }
    return instance;
  }
  public double Get_tx(){
    tx = m_limTable.getEntry("tx").getDouble(0);
    if (RobotBase.isSimulation()){
      simTx = simTx-0.5;
      if( simTx <= 0.0){
        simTx = 0.0;
      }
      tx = simTx; 
    }
    SmartDashboard.putNumber("tx", tx);
    return tx;
  }

  public double Get_ty(){
    ty = m_limTable.getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("ty", ty);
    return ty;
  }

  public double Get_ta(){
    ta = m_limTable.getEntry("ta").getDouble(0);
    return ta;
  }

  public double Get_tv(){
    tv = m_limTable.getEntry("tv").getDouble(0);
    if (RobotBase.isSimulation()){
      simTv = simTv-0.5;
      if( simTv <= 1.0){
        simTv = 1.0;
      }
      tv = simTv; 
    }
    SmartDashboard.putNumber("tv", tv);
    return tv;
  }

  public boolean isTargetVisible() {
    return (Get_tv() == 1.0);
}

  public void setLightMode(int mode){
    m_limTable.getEntry("ledMode").setNumber(mode);
  }

  public double getLightMode(){
    return m_limTable.getEntry("ledMode").getDouble(0);
  }


  public double limelightXPID(double tx){
    double kP = 0.008;
    double correctionMin = 0.003;
    double deadZone = 0.05;
    double correction = tx * kP;
    if(correction < correctionMin) {
      correction = Math.copySign(correctionMin, correction);
    }
    if(Math.abs(tx) < deadZone){
      correction = 0;
    }
    return correction;
  }

  public double limelightYPID(double ty){
    double kP = 0.008;
    double correctionMin = 0.003;
    double deadZone = 0.05;
    double correction = ty * kP;
    if(correction < correctionMin){
      correction = Math.copySign(correctionMin, correction);
    }
    if(Math.abs(ty)< deadZone){
      correction = 0;
    }
    return correction;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum LedMode {
    PIPELINE, OFF, BLINK, ON
  }
  public void reloadLimeLightSimu() {
    simTx = 30 ;
    simTv = 10.0 ;
  }

}
