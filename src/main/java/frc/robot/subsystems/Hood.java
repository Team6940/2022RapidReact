// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.team1678.math.Conversions;
import edu.wpi.first.wpilibj.RobotBase;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private WPI_TalonSRX mHoodmotor;
  private static final double kEncMin = 0.0;//TODO
  private static final double kEncMax = 627;
  private static final double kAngleMin = 20.0;
  private static final double kAngleMax = 70.0;
  private static Hood instance = null;
  private int offset = 0;
  PeriodicIO periodicIO = new PeriodicIO();

  public Hood() {
    mHoodmotor = new WPI_TalonSRX(Constants.HoodMotorPort);

    mHoodmotor.setInverted(false);
    mHoodmotor.setSensorPhase(false);//TODO
    mHoodmotor.setNeutralMode(NeutralMode.Brake);

    configurationOne();
    mHoodmotor.configForwardSoftLimitThreshold(Conversions.degreesToTalon(kEncMin, Constants.HOOD_GEAR_RATIO), 10); //TODO
    mHoodmotor.configReverseSoftLimitThreshold(Conversions.degreesToTalon(kEncMax, Constants.HOOD_GEAR_RATIO), 10); //TODO
    mHoodmotor.configForwardSoftLimitEnable(true, 10);
    mHoodmotor.configReverseSoftLimitEnable(true, 10);

    mHoodmotor.configVoltageCompSaturation(12);
    mHoodmotor.enableVoltageCompensation(true);

    mHoodmotor.configPeakOutputForward(0.50, 10);//TODO
    mHoodmotor.configPeakOutputReverse(-0.50, 10);//TODO

    mHoodmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
  }

  public static Hood getInstance() {
    if (instance == null) {
        instance = new Hood();
    }
    return instance;
  }

  private void configurationOne() {
    mHoodmotor.selectProfileSlot(0, 0);//TODO
    mHoodmotor.config_kP(0, 3.66, 10);
    mHoodmotor.config_kI(0, 0.01, 10);
    mHoodmotor.config_kD(0, 10.0, 10);
    mHoodmotor.config_kF(0, 0.00, 10);
    mHoodmotor.config_IntegralZone(0, 15, 10);
    mHoodmotor.configMotionCruiseVelocity(600, 10);
    mHoodmotor.configMotionAcceleration(1200, 10);
    // mHoodmotor.configMotionSCurveStrength(6);
  }

  public void setHoodAngle(double targetAngle){
    double targetPos = Conversions.degreesToTalon(targetAngle, Constants.HOOD_GEAR_RATIO) + offset;
    periodicIO.demand = (int)targetPos;
    mHoodmotor.set(ControlMode.MotionMagic, targetPos);
  }

  public void setHoodStop(){
    periodicIO.demand = periodicIO.position;
    mHoodmotor.set(ControlMode.PercentOutput, 0);
  }

  public double getHoodAngle(){
    if (RobotBase.isSimulation()){
      return Conversions.talonToDegrees(periodicIO.position - offset, Constants.HOOD_GEAR_RATIO);
    }else{
      return Conversions.talonToDegrees((int) mHoodmotor.getSelectedSensorPosition(0)- offset, Constants.HOOD_GEAR_RATIO);
    }
    
  }

  public void writePeriodicOutputs(){}

  public void outputTelemetry(){
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }

  public void readPeriodicInputs() {
    if (RobotBase.isSimulation())
    {
        periodicIO.position = (int)periodicIO.demand;
    }else{
        periodicIO.position = (int) mHoodmotor.getSelectedSensorPosition(0);  
    }
  }

  public void ZeroHood(){
    mHoodmotor.set(ControlMode.MotionMagic, offset);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class PeriodicIO {

    // Inputs
    public int position;
    // Outputs
    public double demand;
  }
  
}
