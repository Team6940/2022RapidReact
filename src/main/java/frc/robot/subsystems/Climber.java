// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX m_leftclimberMastermotor;
  WPI_TalonFX m_rghtclimberFollowermotor;
  Solenoid m_climbersolenoid;
  private static Climber instance = null;
  PeriodicIO periodicIO = new PeriodicIO();
  private ClimberControlState currentState = ClimberControlState.CLIBER_NOTHING;
  private int num_ElasticClimber = 1;
  private int num_StraightClimber = 1;

  public Climber() {
    m_leftclimberMastermotor = new WPI_TalonFX(Constants.leftClimberMotorPort);
    m_rghtclimberFollowermotor = new WPI_TalonFX(Constants.rghtClimberMotorPort);

    m_rghtclimberFollowermotor.setInverted(true);//TODO
    //m_rghtclimberFollowermotor.follow(m_leftclimberMastermotor);//TODO

    m_leftclimberMastermotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rghtclimberFollowermotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_leftclimberMastermotor.config_kP(0, Constants.kLeftClimberMotorkP);
    m_leftclimberMastermotor.config_kI(0, Constants.kLeftClimberMotorkI);
    m_leftclimberMastermotor.config_kD(0, Constants.kLeftClimberMotorkD);
    m_leftclimberMastermotor.config_IntegralZone(0, Constants.kLeftClimberMotorkIZone);
    m_leftclimberMastermotor.configMotionCruiseVelocity(Constants.LeftClimbermotionCruiseVelocity);
    m_leftclimberMastermotor.configMotionAcceleration(Constants.LeftClimbermotionAcceleration);

    m_rghtclimberFollowermotor.config_kP(0, Constants.kRghtClimberMotorkP);
    m_rghtclimberFollowermotor.config_kI(0, Constants.kRghtClimberMotorkI);
    m_rghtclimberFollowermotor.config_kD(0, Constants.kRghtClimberMotorkD);
    m_rghtclimberFollowermotor.config_IntegralZone(0, Constants.kRghtClimberMotorkIZone);
    m_rghtclimberFollowermotor.configMotionCruiseVelocity(Constants.RghtClimbermotionCruiseVelocity);
    m_rghtclimberFollowermotor.configMotionAcceleration(Constants.RghtClimbermotionAcceleration);

    m_leftclimberMastermotor.configVoltageCompSaturation(12);
    m_leftclimberMastermotor.enableVoltageCompensation(true);

    m_rghtclimberFollowermotor.configVoltageCompSaturation(12);
    m_rghtclimberFollowermotor.enableVoltageCompensation(true);


    m_climbersolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClimberSolenoidPort);
  }

  public static Climber getInstance() {
    if (instance == null){
        instance = new Climber();
    }
    return instance;
  }

  public void writePeriodicOutputs(){
    if(currentState == ClimberControlState.CLIBER_NOTHING){
      m_leftclimberMastermotor.set(ControlMode.PercentOutput, 0);
      m_rghtclimberFollowermotor.set(ControlMode.PercentOutput, 0);
    }else if(currentState == ClimberControlState.AUTO_CLIMB_FORWARD){
      double LeftcurrentPosition = m_leftclimberMastermotor.getSelectedSensorPosition();
      double RghtcurrentPosition = m_rghtclimberFollowermotor.getSelectedSensorPosition();
      double LeftdesiredPostion = LeftcurrentPosition + 2048 *3; //TODO
      double RghtdesiredPostion = RghtcurrentPosition + 2048 *3; //TODO
      m_leftclimberMastermotor.set(ControlMode.MotionMagic, LeftdesiredPostion);
      m_rghtclimberFollowermotor.set(ControlMode.MotionMagic, RghtdesiredPostion);
    }else if(currentState == ClimberControlState.AUTO_CLIMB_REVERSE){
      double LeftcurrentPosition = m_leftclimberMastermotor.getSelectedSensorPosition();
      double RghtcurrentPosition = m_rghtclimberFollowermotor.getSelectedSensorPosition();
      double LeftdesiredPostion = LeftcurrentPosition - 2048 *3; //TODO
      double RghtdesiredPostion = RghtcurrentPosition - 2048 *3; //TODO
      m_leftclimberMastermotor.set(ControlMode.MotionMagic, LeftdesiredPostion);
      m_rghtclimberFollowermotor.set(ControlMode.MotionMagic, RghtdesiredPostion);
    }else if(currentState == ClimberControlState.STRAIGHTCLIMBER_ON){
      m_climbersolenoid.set(true);
    }else if(currentState == ClimberControlState.STRAIGHTCLIMBER_OFF){
      m_climbersolenoid.set(false);
    }
  }

  public void outputTelemetry(){
    SmartDashboard.putNumber("Climer Speed", m_leftclimberMastermotor.getMotorOutputPercent());
  }

  public void autosetElasticClimber(){
    if(num_ElasticClimber % 2 == 1){
      currentState = ClimberControlState.AUTO_CLIMB_FORWARD;
    }else{
      currentState = ClimberControlState.AUTO_CLIMB_REVERSE;
    }
    num_ElasticClimber += 1;
  }

  public void stopElasticClimber(){
    currentState = ClimberControlState.CLIBER_NOTHING;
  }

  public void autosetStraighClimber(){
    if(num_StraightClimber % 2 == 1){
      currentState = ClimberControlState.STRAIGHTCLIMBER_ON;
    }else{
      currentState = ClimberControlState.STRAIGHTCLIMBER_OFF;
    }
    num_StraightClimber += 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ClimberControlState {
    CLIBER_NOTHING,
    ELASTICCLIMBER_FORWARD,
    ELASTICCLIMBER_REVERSE,
    STRAIGHTCLIMBER_ON,
    STRAIGHTCLIMBER_OFF,
    AUTO_CLIMB_FORWARD,
    AUTO_CLIMB_REVERSE
  }

  public static class PeriodicIO {
      // Inputs
      public double percentoutput;

      // Outputs
      public double demand;
  }
}
