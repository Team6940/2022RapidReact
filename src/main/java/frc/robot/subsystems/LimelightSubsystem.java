// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  private static LimelightSubsystem instance = null;
  /** Creates a new Limelight. */
  public NetworkTable m_limTable;

  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;
  
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
    return tv;
  }

  public boolean isTargetVisible() {
    return (Get_tv() == 1.0);
}

  public void setLightMode(int mode){
    m_limTable.getEntry("ledMode").setNumber(mode);
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

  public double getShooterLaunchVelocity(double shooterAngle) {
    double speed = 0;
    double d = Constants.CARGO_DIAMETER;
    double D = Constants.UPPER_HUB_DIAMETER;
		double g = 9.81;
		double H = Constants.LL_UPPER_HUB_HEIGHT;
    double h = Constants.SHOOTER_MOUNT_HEIGHT;
		double L = 10 + Constants.UPPER_HUB_DIAMETER / 2; //getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2 ; 
		double alpha = Math.toRadians(shooterAngle); // Set to proper value
    /* v is mini speed  */
    double vMin = Math.sqrt(g * (H-h+Math.sqrt(Math.pow(L,2)+Math.pow(H-h,2))));
		double v = L / Math.cos(alpha) * Math.sqrt( g / (2 * ( L *Math.tan(alpha) - H + h )));
    double beta = Math.toDegrees(Math.atan(Math.tan(alpha) - 2*(H-h) / L));
    double betaMinLimit = Math.toDegrees(Math.asin(d/D));
    if( (v >= vMin) && (beta >= betaMinLimit )){
      speed = v;
    }
    SmartDashboard.putNumber("calcShootSpeed", speed);
    return speed;
	}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
