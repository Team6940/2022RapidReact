// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.team1706.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class LimelightSubsystem extends SubsystemBase {

  private static LimelightSubsystem instance = null;
  /** Creates a new Limelight. */
  public NetworkTable m_limTable;
  public double constTx = 4.0;
  public double constTv = 4.0;
  public double constTy = 4.0;
  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;
  public double simTx = constTx ;
  public double simTy = constTy ;
  public double simTv = constTv ;
  public int simuTxStop = 0;
  public OptionalDouble distancetoTarget = OptionalDouble.empty();

  private static Point2D[] points = new Point2D.Double[] {
    // (ty-angle,distance)
    new Point2D.Double(-24.0, 7.366/*290.0*/), // 242
    new Point2D.Double(-20.0, 6.045/*238.0*/), // 196
    new Point2D.Double(-17.5, 5.258/*207.0*/), // 163
    new Point2D.Double(-15.0, 4.724/*186.0*/), // 141
    new Point2D.Double(-12.5, 4.293/*169.0*/), // 121
    new Point2D.Double(-10.0, 3.912/*154.0*/), // 107
    new Point2D.Double(-5.0, 3.404/*134.0*/), // 96
    new Point2D.Double(0.0, 2.946/*116.0*/), // 85
    new Point2D.Double(5.0, 2.642/*104.0*/), // 77
    new Point2D.Double(10.0, 2.337/*92.0*/),
    new Point2D.Double(15.0, 2.108/*83.0*/),
    new Point2D.Double(20.0, 1.905/*75.0*/)
    //
};
private static LinearInterpolationTable distTable = new LinearInterpolationTable(points);

  
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
      if(simTx >= 0.2){
        simTx = simTx-0.2;
      }
      tx = simTx; 
    }
    SmartDashboard.putNumber("Debug/Limglight/tx", tx);
    return tx;
  }

  public double Get_ty(){
    ty = m_limTable.getEntry("ty").getDouble(0);
    if (RobotBase.isSimulation()){
      if(simTy >= 0.2){
        simTy = simTy-0.2;
      }
      ty = simTy; 
    }
    SmartDashboard.putNumber("Debug/Limglight/ty", ty);
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
    SmartDashboard.putNumber("Debug/Limglight/tv", tv);
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

  
  public double getRobotToTargetDistance() {
		return (Constants.LL_UPPER_HUB_HEIGHT - Constants.LL_MOUNT_HEIGHT)
             / Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + Get_ty()));
	}

  public OptionalDouble getRobotToTargetDistance_Opt(){
    distancetoTarget = OptionalDouble.of(
      (Constants.LL_UPPER_HUB_HEIGHT - Constants.LL_MOUNT_HEIGHT)
      / Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + Get_ty())));
    return distancetoTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum LedMode {
    PIPELINE, OFF, BLINK, ON
  }
  public void reloadLimeLightSimu() {
    simTx = constTx ;
    simTy = constTy;
    simTv = constTv ;
    simuTxStop = 0;
  }

  /**
     * Uses tuned interpolation table to report distance
     *
     * @return the distance to the target in inches
  */
  public double getDistance() {  //TODO
      final double tx = Get_tx();
      final double tyAdj = (Get_ty() -0.0084125*tx*tx)/(0.000267*tx*tx+1.0); //New geometric correction function
      final double distance = distTable.getOutput(tyAdj);
      SmartDashboard.putNumber("Limelight ty", Get_ty());
      // SmartDashboard.putNumber("LimelightDistance", distance);
      return distance;
  }

}
