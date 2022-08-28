// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberNew extends SubsystemBase {
  /** Creates a new ClimberNew. */
  private static ClimberNew instance ;
  private Solenoid pivotSolenoid;
  private WPI_TalonFX climberMotorLeft;
  private WPI_TalonFX climberMotorRght;

  public static ClimberNew getInstance() {
    if (instance == null){
        instance = new ClimberNew();
    }
    return instance;
  }

  public ClimberNew() {
    //super(Constants.CLIMBER_PERIOD, 1);

    climberMotorLeft = new WPI_TalonFX(Constants.leftClimberMotorPort);
    climberMotorRght = new WPI_TalonFX(Constants.rghtClimberMotorPort);

    climberMotorLeft.config_kF(0, Constants.CLIMBER_MOTOR_KF);
    climberMotorLeft.config_kP(0, Constants.CLIMBER_MOTOR_KP);//TODO
    climberMotorLeft.config_kI(0, Constants.CLIMBER_MOTOR_KI);
    climberMotorLeft.config_kD(0, Constants.CLIMBER_MOTOR_KD);
    climberMotorLeft.config_IntegralZone(0, Constants.CLIMBER_MOTOR_IZONE);
    climberMotorLeft.configPeakOutputForward(Constants.CLIMBER_MOTOR_MAX_OUTPUT);
    climberMotorLeft.configPeakOutputReverse(-Constants.CLIMBER_MOTOR_MAX_OUTPUT);
    climberMotorLeft.setNeutralMode(NeutralMode.Brake);

    climberMotorRght.config_kF(0, Constants.CLIMBER_MOTOR_KF);
    climberMotorRght.config_kP(0, Constants.CLIMBER_MOTOR_KP);//TODO
    climberMotorRght.config_kI(0, Constants.CLIMBER_MOTOR_KI);
    climberMotorRght.config_kD(0, Constants.CLIMBER_MOTOR_KD);
    climberMotorRght.config_IntegralZone(0, Constants.CLIMBER_MOTOR_IZONE);
    climberMotorRght.configPeakOutputForward(Constants.CLIMBER_MOTOR_MAX_OUTPUT);
    climberMotorRght.configPeakOutputReverse(-Constants.CLIMBER_MOTOR_MAX_OUTPUT);
    climberMotorRght.setNeutralMode(NeutralMode.Brake);

    climberMotorLeft.configMotionAcceleration(8000);
    climberMotorLeft.configMotionCruiseVelocity(10000);
    climberMotorRght.configMotionAcceleration(8000);
    climberMotorRght.configMotionCruiseVelocity(10000);

    pivotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PIVOT_SOLENOID_ID);

    climberMotorLeft.setInverted(false);
    climberMotorRght.setInverted(true);

    //climberMotorLeft.setSelectedSensorPosition(0);//TODO ???
  }

  public void outputTelemetry(){
    //SmartDashboard.putNumber("Debug/Intake/Current", intakeMotor.getStatorCurrent());
    SmartDashboard.putBoolean("Debug/Climber/ClimberSolState: ", pivotSolenoid.get());
    SmartDashboard.putString("Debug/Climber/ClimberSolState", wantedClimberSolState.toString());
    SmartDashboard.putString("Debug/Climber/WantedClimber State", getWantedClimberState().name());
  }
  
  // Intake States
  public enum ClimberSolState {
    OPEN, CLOSE
  }

  private ClimberSolState wantedClimberSolState = ClimberSolState.CLOSE;

  // this a extern func for other command call.
  public synchronized void setClimberSolState(ClimberSolState climberSolState) {
    wantedClimberSolState = climberSolState;
    switch (climberSolState) {
      case OPEN:
        pivotSolenoid.set(true);
        break;
      case CLOSE:
        pivotSolenoid.set(false);
    }
  }
  
  public enum ClimberState {
    PUSH, PULL, INIT
  }

  private ClimberState wantedClimberState = ClimberState.INIT;

  // this a extern func for other command call.
  public synchronized void setWantedClimberState(ClimberState climberState) {
    wantedClimberState = climberState;
  }
    
  public synchronized ClimberState getWantedClimberState() {
    return wantedClimberState ;
  }

  private void setClimberLeftMotor(double position) {
    climberMotorLeft.set(ControlMode.MotionMagic, position);
  }

  private void setClimberRghtMotor(double position) {
    climberMotorRght.set(ControlMode.MotionMagic, position);
  }

  private void setClimberState(ClimberState climberState) {
      switch (climberState) {
          case PUSH:
            setClimberLeftMotor(150000);//TODO
            setClimberRghtMotor(150000);
            break;
          case PULL:
            setClimberLeftMotor(30000);
            setClimberRghtMotor(30000); 
            break;
          case INIT:
            setClimberLeftMotor(0);
            setClimberRghtMotor(0);
            break;
      }
  }

  private int cnt = 0;
  public void autoturnclimber()
  {
    if (cnt % 2 == 0) {
      setWantedClimberState(ClimberState.PUSH);
    }else{
      setWantedClimberState(ClimberState.PULL);
    }
    cnt++;
  }
  
  private int count = 0;
  public void autoturnclimberSol(){
    if(count % 2 == 0){
      setClimberSolState(ClimberSolState.OPEN);
    }
    else{
      setClimberSolState(ClimberSolState.CLOSE);
    }
    count++;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setClimberState(wantedClimberState);
    outputTelemetry();
  }
}
