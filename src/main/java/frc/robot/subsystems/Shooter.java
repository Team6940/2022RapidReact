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
import frc.robot.lib.team503.util.Util;
import frc.robot.lib.team1678.math.Conversions;



public class Shooter extends SubsystemBase {
  
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooterSpeed = new InterpolatingTreeMap<>();
    private static Shooter instance = null;
    private int num = 1;

    static { //TODO first is meters fou distance,second is RPM
        kDistanceToShooterSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1000.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(1.9304), new InterpolatingDouble(2400.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(2.540), new InterpolatingDouble(2800.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(3.302), new InterpolatingDouble(3500.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(4.064), new InterpolatingDouble(3700.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(4.826), new InterpolatingDouble(3800.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(6.096), new InterpolatingDouble(3900.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(7.620), new InterpolatingDouble(4000.0));
    }
    private static WPI_TalonFX mShooter;
    private final PeriodicIO periodicIO = new PeriodicIO();
    ControlState state = ControlState.OPEN_LOOP;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private double mTargetVelocity = 0;
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

        lMasterConfig.slot0.kP = 1.5;
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

    public ControlState getState() {
      return state;
    }

    public void setState(ControlState newState) {
        state = newState;
    }

    private double getLastSetVelocity() {
        return mTargetVelocity;
    }

    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber("Flywheel Velocity", periodicIO.flywheel_velocity);
            SmartDashboard.putNumber("Flywheel Current", periodicIO.flywheel_current);
            SmartDashboard.putNumber("Flywheel Goal", periodicIO.flywheel_demand);
            SmartDashboard.putNumber("Flywheel Temperature", periodicIO.flywheel_temperature);
            SmartDashboard.putBoolean("Shooter Spun Up: ", spunUp());
            SmartDashboard.putNumber("Shooter Master Voltage", periodicIO.flywheel_voltage);
            SmartDashboard.putNumber("Shooter Slave Voltage", periodicIO.slave_voltage);
        }
    }

    public void stop() {
        setState(ControlState.OPEN_LOOP);
        periodicIO.flywheel_demand = 0.0;
        velocityStabilized = true;
    }

    public void onStart(double timestamp) {
        setState(ControlState.OPEN_LOOP);
    }

    public void onLoop(double timestamp) {

    }

    public void onStop(double timestamp) {
    }

    /**
     * Run the shooter in open loop/raw voltage input
     */
    public void setOpenLoop(double shooterPercent) {
        setState(ControlState.OPEN_LOOP);
        periodicIO.flywheel_demand = shooterPercent;
    }

    public  boolean spunUp() {
    if (periodicIO.flywheel_demand > 0) {
        return Util.epsilonEquals(FalconToMeterSpeed(periodicIO.flywheel_demand)
            , periodicIO.flywheel_velocity
            , Constants.kShooterTolerance);
    }
    return false;
    }
    /**
    * Update the velocity control setpoint of the Shooter. This is the main method
    * to call for velocity control Request
    */
    public void setVelocity(double meterSpeed) {
      setState(ControlState.VELOCITY);
      mTargetVelocity = meterSpeed;
      periodicIO.flywheel_demand = meterSpeed /
      (Constants.kFlyWheelWheelDiameter * Math.PI) /
      Constants.kFlyWheelEncoderReductionRatio *
      2048.0 * 0.1;
      
      //periodicIO.flywheel_demand = (rpm / kFlywheelRevsPerMotor) * RobotState.getInstance().getShooterSpeedIncrement()
    }

    public void aimShooterSpeedWithVision() {
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        setState(ControlState.VISION);
        periodicIO.flywheel_demand = 0.0;
        // spinUpStage = true;
    }

    public void aimShooterSpeedToDistance(double distance) {
        setState(ControlState.DISTANCE);
        // periodicIO.flywheel_demand = distance;
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        // spinUpStage = true;

    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed.getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 300.0), 0.0))).value;
    }

    public boolean isShooterReady() {
        return this.velocityStabilized;
    }

    public void idle() {
        setState(ControlState.IDLE);
        mTargetVelocity = Constants.kFlywheelIdleVelocity;
        periodicIO.flywheel_demand = MeterSpeedToFalcon(Constants.kFlywheelIdleVelocity);
    }

    public void readPeriodicInputs() {
        periodicIO.timestamp = Timer.getFPGATimestamp();
            
        periodicIO.flywheel_velocity = getFlywheelVelocity();
        periodicIO.flywheel_voltage = mShooter.getMotorOutputVoltage();
        periodicIO.flywheel_current = mShooter.getSupplyCurrent();
        periodicIO.flywheel_temperature = mShooter.getTemperature();

    }

    public void writePeriodicOutputs() {
        if (getState() == ControlState.OPEN_LOOP) {
            mShooter.set(ControlMode.PercentOutput, periodicIO.flywheel_demand);
        } else if (getState() == ControlState.Table_Shoot) {
            double targetRpm = getShooterSpeedForDistance(LimelightSubsystem.getInstance().getRobotToTargetDistance());
            double targetVelocity = RpmToMeterSpeed(targetRpm);
            if (LimelightSubsystem.getInstance().Get_tv() == 0.0) {
                targetVelocity = Constants.kFlyWheelWheelDefaultSpeed;  //TODO
            }
            periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
            velocityStabilized = meterSpeedToRpm(targetVelocity) - getShooterSpeedRpm() < 50; //TODO
            mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand); 
        } else if (getState() == ControlState.DISTANCE) {
            double targetRpm =  getShooterSpeedForDistance(Conversions.inchesToMeters(167.0)); //TODO
            double targetVelocity = RpmToMeterSpeed(targetRpm);

            periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);

            mShooter.set(ControlMode.Velocity,periodicIO.flywheel_demand); 
        } else if(getState() == ControlState.Algorithm_Shoot){
            double targetVelocity = getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE);
            if (LimelightSubsystem.getInstance().Get_tv() == 0.0) {
                targetVelocity = Constants.kFlyWheelWheelDefaultSpeed;  //TODO
            }
            periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
            mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand);
        }
        else {
            mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand); 
        }
    }

    public void testShooter(double power, boolean input1, boolean input2) {
      mShooter.set(ControlMode.PercentOutput, power);
    }

    public void shootFlywheel(double meterSpeed) {
        // meters per second
        double driveOutput = meterSpeed 
        / (Constants.kFlyWheelWheelDiameter * Math.PI)
        / Constants.kFlyWheelEncoderReductionRatio
        * 2048.0 * 0.1;
        mShooter.set(ControlMode.Velocity,driveOutput);
    }

    public void setFlywheelOutput(double percentage) {
        mShooter.set(ControlMode.PercentOutput, percentage);
    }

    public void stopFlywheel() {
        mShooter.set(ControlMode.Disabled, 0);
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

    public void resetFlywheelPosition() {
        mShooter.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public boolean isFlywheelAtTargetVelocity() {
        //return MathUtils.epsilonEquals(
        //        getFlywheelVelocity(),
        //        getFlywheelTargetVelocity(),
        //        Constants.FLYWHEEL_ALLOWABLE_ERROR  //TODO
        //);
        return true;  //TODO
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
            state = ControlState.Algorithm_Shoot;
          }
          else{
            state = ControlState.Table_Shoot;
          }
          num += 1;
    }

    public enum ControlState {
        IDLE, OPEN_LOOP, VELOCITY, VISION, DISTANCE, Algorithm_Shoot, Table_Shoot
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
