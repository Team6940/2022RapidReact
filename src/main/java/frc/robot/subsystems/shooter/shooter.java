package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
import frc.robot.Constants;

public class shooter extends SubsystemBase {

  private static WPI_TalonFX leftMaster;
  private static WPI_TalonFX rightSlave;
  private static boolean isArmed = false;
  private static int targetShooterSpeed = 0;
  private static int shooterStableCounts = 0;

  public shooter() {
    configTalons();
  }

  private void configTalons() {
    TalonFXConfiguration lMasterConfig = new TalonFXConfiguration();

    lMasterConfig.supplyCurrLimit.currentLimit = 40.0;
    lMasterConfig.supplyCurrLimit.triggerThresholdCurrent = 50.0;
    lMasterConfig.supplyCurrLimit.triggerThresholdTime = 2.0;
    lMasterConfig.supplyCurrLimit.enable = true;
    lMasterConfig.slot0.kP = 1.5;
    lMasterConfig.slot0.kI = 0;
    lMasterConfig.slot0.kD = 5;
    lMasterConfig.slot0.kF = 0.048;
    lMasterConfig.peakOutputForward = 1.0;
    lMasterConfig.peakOutputReverse = 0.0;
    lMasterConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
    lMasterConfig.velocityMeasurementWindow = 16;
    leftMaster = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);
    leftMaster.configAllSettings(lMasterConfig);
    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftMaster.enableVoltageCompensation(false);

    rightSlave = new WPI_TalonFX(Constants.SHOOT_R_MASTER_ID);
    rightSlave.configAllSettings(lMasterConfig);
    rightSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setInverted(true);
    rightSlave.enableVoltageCompensation(false);
    rightSlave.follow(leftMaster);
  }


  public void run(int velocity) {
    rightSlave.follow(leftMaster);
    leftMaster.set(ControlMode.Velocity, velocity);
    targetShooterSpeed = velocity;
    //logger.info("Running closed loop at: {}", velocity);
  }

  public void stop() {
    rightSlave.follow(leftMaster);
    leftMaster.set(ControlMode.PercentOutput, 0);
    //logger.info("Stopping Shooter (open loop)");
  }

  public void setArmedState(boolean armed) {
    isArmed = armed;
  }

  public boolean isArmed() {
    return isArmed;
  }

  public void runOpenLoop(double percent) {
    rightSlave.follow(leftMaster);
    leftMaster.set(ControlMode.PercentOutput, percent);
    //logger.info("Running open loop at: {}", percent);
  }

  public double getShooterSpeed() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public boolean atTargetSpeed() {
    double currentSpeed = leftMaster.getSelectedSensorVelocity();
    if (Math.abs(targetShooterSpeed - currentSpeed) > Constants.kCloseEnough) {
      shooterStableCounts = 0;
    } else {
      shooterStableCounts++;
    }
    if (shooterStableCounts >= Constants.kStableCounts) {
      //logger.info("Shooter at speed {}", targetShooterSpeed);
      return true;
    } else {
      return false;
    }
  }
}
