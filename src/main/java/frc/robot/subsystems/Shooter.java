package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.team503.util.InterpolatingDouble;
import frc.robot.lib.team503.util.InterpolatingTreeMap;



public class Shooter extends SubsystemBase {
  
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooterSpeed = new InterpolatingTreeMap<>();
    private static Shooter instance = null;
    private int num = 1;
    private int shootMode = 1;//1 means table shoot ,0 means algorithm shoot

    static { //TODO first is meters fou distance,second is RPM
        kDistanceToShooterSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(2.597050));//TODO
        kDistanceToShooterSpeed.put(new InterpolatingDouble(1.9304), new InterpolatingDouble(6.232920));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(2.540), new InterpolatingDouble(7.271740));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(3.302), new InterpolatingDouble(9.089675));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(4.064), new InterpolatingDouble(9.609085));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(4.826), new InterpolatingDouble(9.868790));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(6.096), new InterpolatingDouble(10.128495));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(7.620), new InterpolatingDouble(10.388200));
    }
    private static WPI_TalonFX mShooter;
    private final PeriodicIO periodicIO = new PeriodicIO();
    ShooterControlState currentState = ShooterControlState.STOP;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private boolean velocityStabilized = true;

    public Shooter() {
        configTalons();
    }

    public static Shooter getInstance() {
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }
    private void configTalons() {
        TalonFXConfiguration lMasterConfig = new TalonFXConfiguration();

        lMasterConfig.slot0.kP = 1.5;//TODO
        lMasterConfig.slot0.kI = 0;
        lMasterConfig.slot0.kD = 5;
        lMasterConfig.slot0.kF = 0.048;
        lMasterConfig.peakOutputForward = 1.0;
        lMasterConfig.peakOutputReverse = 0.0;
        mShooter = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);
        mShooter.configAllSettings(lMasterConfig);
        mShooter.setNeutralMode(NeutralMode.Coast);
        mShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    public ShooterControlState getState() {
      return currentState;
    }

    public void setState(ShooterControlState newState) {
        currentState = newState;
    }

    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber("Shooter Mode", shootMode);
            SmartDashboard.putNumber("Flywheel Velocity", periodicIO.flywheel_velocity);
            SmartDashboard.putNumber("Flywheel Current", periodicIO.flywheel_current);
            SmartDashboard.putNumber("Flywheel Goal", periodicIO.flywheel_demand);
            SmartDashboard.putNumber("Flywheel Temperature", periodicIO.flywheel_temperature);
            SmartDashboard.putNumber("Shooter Master Voltage", periodicIO.flywheel_voltage);
            SmartDashboard.putNumber("Shooter Slave Voltage", periodicIO.slave_voltage);
            SmartDashboard.putNumber("RPM1000", RpmToMeterSpeed(1000));
            SmartDashboard.putNumber("RPM2400", RpmToMeterSpeed(2400));
            SmartDashboard.putNumber("RPM2800", RpmToMeterSpeed(2800));
            SmartDashboard.putNumber("RPM3500", RpmToMeterSpeed(3500));
            SmartDashboard.putNumber("RPM3700", RpmToMeterSpeed(3700));
            SmartDashboard.putNumber("RPM3800", RpmToMeterSpeed(3800));
            SmartDashboard.putNumber("RPM3900", RpmToMeterSpeed(3900));
            SmartDashboard.putNumber("RPM4000", RpmToMeterSpeed(4000));

        }
    }

    /**
    * Update the velocity control setpoint of the Shooter. This is the main method
    * to call for velocity control Request
    */
    public void setVelocity(double meterSpeed) {
      periodicIO.flywheel_demand = meterSpeed /
      (Constants.kFlyWheelWheelDiameter * Math.PI) /
      Constants.kFlyWheelEncoderReductionRatio *
      2048.0 * 0.1;
    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed.getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 7.62), 0.0))).value;
    }

    public boolean isShooterReady() {
        return this.velocityStabilized;
    }

    public void readPeriodicInputs() {
        periodicIO.timestamp = Timer.getFPGATimestamp();
            
        periodicIO.flywheel_velocity = getFlywheelVelocity();
        periodicIO.flywheel_voltage = mShooter.getMotorOutputVoltage();
        periodicIO.flywheel_current = mShooter.getSupplyCurrent();
        periodicIO.flywheel_temperature = mShooter.getTemperature();

    }

    public void writePeriodicOutputs() {
        if(currentState == ShooterControlState.STOP) {
            mShooter.set(ControlMode.PercentOutput, 0);
            if(Turret.getInstance().isVisionFinding() || Turret.getInstance().isVisionMoving()){
                currentState = ShooterControlState.PREPARE_SHOOT;
            }else if(Turret.getInstance().isTargetReady()){
                currentState = ShooterControlState.SHOOT;
            }
        }else if(currentState == ShooterControlState.PREPARE_SHOOT){
            setVelocity(Constants.kFlywheelIdleVelocity);
            if(Turret.getInstance().isTargetReady()){
                currentState = ShooterControlState.SHOOT;
            }else if(Turret.getInstance().isStop()){
                currentState = ShooterControlState.STOP;
            }
        }else if(currentState == ShooterControlState.SHOOT){
            if(shootMode == 1){
                double targetVelocity = getShooterSpeedForDistance(LimelightSubsystem.getInstance().getRobotToTargetDistance());
                periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
                velocityStabilized = meterSpeedToRpm(targetVelocity) - getShooterSpeedRpm() < 50; //TODO
                mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand); 
            }else{
                double targetVelocity = getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE);
                periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
                mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand);
            }
            if(Turret.getInstance().isStop()){
                currentState = ShooterControlState.STOP;
            }else if(Turret.getInstance().isVisionFinding() || Turret.getInstance().isVisionMoving()){
                currentState = ShooterControlState.PREPARE_SHOOT;
            }
        }
    }

    public void shootFlywheel(double meterSpeed) {
        // meters per second
        double driveOutput = meterSpeed 
        / (Constants.kFlyWheelWheelDiameter * Math.PI)
        / Constants.kFlyWheelEncoderReductionRatio
        * 2048.0 * 0.1;
        mShooter.set(ControlMode.Velocity,driveOutput);
    }

    public boolean shootIsReady(){
        return (currentState == ShooterControlState.SHOOT);
    }

    public double getFlywheelVelocity() {
        // meters per second
        double speed = 0;
        speed = mShooter.getSelectedSensorVelocity() / 0.1 / 2048.0 
                * Constants.kFlyWheelEncoderReductionRatio
                * Constants.kFlyWheelWheelDiameter * Math.PI;
        return speed;
    }

    public double getFlywheelTargetVelocity() {
        // meters per second
        double speed = 0;
        speed = mShooter.getClosedLoopTarget() / 0.1 / 2048.0 
        * Constants.kFlyWheelEncoderReductionRatio
        * Constants.kFlyWheelWheelDiameter * Math.PI;
        return speed;
    }

    public synchronized double getShooterSpeedRpm() {
        return mShooter.getSelectedSensorVelocity() * 600.0 / 2048.0;
    }

    public double meterSpeedToRpm(double meterSpeed){
        // cycles per minnute
        double rpm = meterSpeed / (Math.PI * Constants.kFlyWheelWheelDiameter) 
                     / Constants.kFlyWheelEncoderReductionRatio
                     * 60;
        return rpm;
    }

    public double RpmToMeterSpeed(double rpm){
        // meters per second
        double meterSpeed = rpm * Constants.kFlyWheelEncoderReductionRatio
                            * Math.PI * Constants.kFlyWheelWheelDiameter
                            / 60 ;
        return meterSpeed;
    }
    public double MeterSpeedToFalcon(double meterSpeed){
        // meters per second
        double FalconUnits = meterSpeed 
        / (Constants.kFlyWheelWheelDiameter * Math.PI)
        / Constants.kFlyWheelEncoderReductionRatio
        * 2048.0 * 0.1;;
        return FalconUnits;
    }
    public double FalconToMeterSpeed(double falconUnits){
        // meters per second
        double speed = 0;
        speed = falconUnits / 0.1 / 2048.0 
        * Constants.kFlyWheelEncoderReductionRatio
        * Constants.kFlyWheelWheelDiameter * Math.PI;
        return speed;
    }
    public double getRobotToTargetDistance() {
		return (Constants.LL_UPPER_HUB_HEIGHT - Constants.LL_MOUNT_HEIGHT)
             / Math.tan(Math.toRadians(Constants.LL_MOUNT_ANGLE + LimelightSubsystem.getInstance().Get_ty()));
	}

    public double getShooterLaunchVelocity(double shooterAngle) {
        double speed = 0;
        double d = Constants.CARGO_DIAMETER;
        double D = Constants.UPPER_HUB_DIAMETER;
        double g = 9.81;
        double H = Constants.LL_UPPER_HUB_HEIGHT;
        double h = Constants.SHOOTER_MOUNT_HEIGHT;
        double L = getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2; //getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2 ; 
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

    public void autoSwitchShooterMode(){
        if(num % 2 == 1){
            shootMode = 1;
          }
          else{
            shootMode = 0;
          }
          num += 1;
    }

    public void setPrepareShoot(){
        currentState = ShooterControlState.PREPARE_SHOOT;
    }

    public enum ShooterControlState {
        STOP, PREPARE_SHOOT, SHOOT
    }

    public class PeriodicIO {
        public double timestamp;
        
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double slave_voltage;
        public double flywheel_current;
        public double flywheel_temperature;

        //OUTPUTS
        public double flywheel_demand;
        public double kickerDemand;
    }
}
