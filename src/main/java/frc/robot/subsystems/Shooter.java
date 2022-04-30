package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.team2910.math.Vector2;
import frc.robot.lib.team2910.util.InterpolatingDouble;
import frc.robot.lib.team2910.util.InterpolatingTreeMap;
import frc.robot.lib.team6940.math.MathUtils;



public class Shooter extends SubsystemBase {
  
    static InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();
    private static Shooter instance = null;
    private int num = 1;
    private int shootMode = 1;//1 means table shoot ,0 means algorithm shoot
    private boolean ShotWhileMove = true;
    private static WPI_TalonFX mShooter;
    private final PeriodicIO periodicIO = new PeriodicIO();
    ShooterControlState currentState = ShooterControlState.STOP;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private boolean velocityStabilized = true;
    /**
     * We use FeedForward and a little P gains for Shooter
     */
    SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);
    private ShuffleboardTab shooterSpeedTab = Shuffleboard.getTab("Shooter Speed");
    private NetworkTableEntry shooterSpeed =
        shooterSpeedTab.add("Shooter Speed", 1).getEntry();

    private ShuffleboardTab HoodAngleTab = Shuffleboard.getTab("Hood Angle");
    private NetworkTableEntry hoodAngle =
        HoodAngleTab.add("Hood Angle", 1).getEntry();

    static { //TODO first is meters for distance,second is MPS
        SHOOTER_TUNING.put(new InterpolatingDouble(0.0000), new Vector2(20,2.5));//TODO
        SHOOTER_TUNING.put(new InterpolatingDouble(1.9304), new Vector2(35,3.0));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.5400), new Vector2(40,3.5));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.3020), new Vector2(45,4.0));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0640), new Vector2(50,4.5));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.8260), new Vector2(55,5.0));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0960), new Vector2(60,5.5));
        SHOOTER_TUNING.put(new InterpolatingDouble(7.6200), new Vector2(65,6.0));
    }

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
        lMasterConfig.slot0.kD = 0;
        lMasterConfig.slot0.kF = 0;
        lMasterConfig.peakOutputForward = 1.0;
        lMasterConfig.peakOutputReverse = 0.0;
        mShooter = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);
        mShooter.configAllSettings(lMasterConfig);
        mShooter.setNeutralMode(NeutralMode.Coast);
        mShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mShooter.configVoltageCompSaturation(12);
        mShooter.enableVoltageCompensation(true);
    }

    public ShooterControlState getState() {
      return currentState;
    }

    public void setState(ShooterControlState newState) {
        currentState = newState;
    }

    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            SmartDashboard.putString("Shooter State", currentState.name());   
            SmartDashboard.putNumber("Shooter Mode", shootMode);
            SmartDashboard.putNumber("Shooter Velocity", periodicIO.flywheel_velocity);
            SmartDashboard.putBoolean("Shooter ready",shootIsReady());
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
            Hood.getInstance().setHoodStop();
        }else if(currentState == ShooterControlState.INIT) {
            mShooter.set(ControlMode.PercentOutput, 0);
            if(Turret.getInstance().isVisionFinding() || Turret.getInstance().isVisionMoving()){
                currentState = ShooterControlState.PREPARE_SHOOT;
            }else if(Turret.getInstance().isTargetReady()){
                currentState = ShooterControlState.SHOOT;
            }
        }else if(currentState == ShooterControlState.PREPARE_SHOOT){
            //setVelocity(Constants.kFlywheelIdleVelocity);
            double cal_shooterFeedForward = shooterFeedForward.calculate(FalconToMeterSpeed(Constants.kFlywheelIdleVelocity));
            mShooter.set(ControlMode.Velocity, Constants.kFlywheelIdleVelocity, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
            if(Turret.getInstance().isTargetReady()){
                currentState = ShooterControlState.SHOOT;
            }else if(Turret.getInstance().isStop()){
                currentState = ShooterControlState.STOP;
            }
        }else if(currentState == ShooterControlState.SHOOT){
            Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(LimelightSubsystem.getInstance().getRobotToTargetDistance_Opt().orElse(0.0)));
            if(shootMode == 1){
                if(ShotWhileMove){
                    double[] mShotParams = new double [3];
                    mShotParams = GetMovingShotParams(
                        angleAndSpeed.y,                             // Shot Speed
                        angleAndSpeed.x,                             // Hood Angle
                        Turret.getInstance().getAngle(),             // Turret Angle
                        SwerveDriveTrain.getInstance().GetVxSpeed(), // Swerve speed in X axis (field-oriented)
                        SwerveDriveTrain.getInstance().GetVySpeed());// Swerve speed in Y axis (field-oriented)
                    double targetVelocity = mShotParams[2];
                    double targetHoodAngle = mShotParams[1];
                    double targetTurretAngle = mShotParams[0];
                    double cal_shooterFeedForward = shooterFeedForward.calculate(targetVelocity);
                    Hood.getInstance().setHoodAngle(targetHoodAngle);
                    Turret.getInstance().setTurretAngle(targetTurretAngle);
                    mShooter.set(ControlMode.Velocity, targetVelocity, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
                }else{
                    //double targetVelocity = getShooterSpeedForDistance(LimelightSubsystem.getInstance().getRobotToTargetDistance());
                    double targetVelocity = angleAndSpeed.y;
                    double targetAngle = angleAndSpeed.x;
                    double cal_shooterFeedForward = shooterFeedForward.calculate(targetVelocity);
                    periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
                    velocityStabilized = meterSpeedToRpm(targetVelocity) - getShooterSpeedRpm() < 50; //TODO
                    Hood.getInstance().setHoodAngle(targetAngle);
                    mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
                }
            }else{
                double targetVelocity = getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE);
                double cal_shooterFeedForward = shooterFeedForward.calculate(targetVelocity);
                periodicIO.flywheel_demand = MeterSpeedToFalcon(targetVelocity);
                mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
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

    public double getShooterLaunchVelocity(double shooterAngle) {
        double speed = 0;
        double d = Constants.CARGO_DIAMETER;
        double D = Constants.UPPER_HUB_DIAMETER;
        double g = 9.81;
        double H = Constants.LL_UPPER_HUB_HEIGHT;
        double h = Constants.SHOOTER_MOUNT_HEIGHT;
        double L = LimelightSubsystem.getInstance().getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2; //getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2 ; 
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

    public void setStopShoot(){
        currentState = ShooterControlState.STOP;
    }

    public void setInitShoot(){
        currentState = ShooterControlState.INIT;
    }

    public double readShooterSpeedFromShuffleBoard(){
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed.getDouble(1.0));
        return shooterSpeed.getDouble(1.0);
    }

    public double readHoodAngleFromShuffleBoard(){
        SmartDashboard.putNumber("Hood Angle", hoodAngle.getDouble(20.0));
        return hoodAngle.getDouble(20.0);
    }

    public double[] GetMovingShotParams(double V, double phi_v, double phi_h, double vx, double vy){
        double theta_h = Math.atan((V*Math.cos(phi_v)*Math.sin(phi_h) + vy) /
                        (V*Math.cos(phi_v)*Math.cos(phi_h) + vx));
        double theta_v = MathUtils.acot((V*Math.cos(phi_v)*Math.cos(phi_h) +
                          vx) / (V*Math.sin(phi_v)*Math.cos(theta_h)));
        double vs = V*Math.sin(phi_v)/Math.sin(theta_v);

        double[] myShootParams = new double [3];
        myShootParams[0] = theta_h;
        myShootParams[1] = theta_v;
        myShootParams[2] = vs;

        return myShootParams;
    }

    public enum ShooterControlState {
        STOP, INIT,PREPARE_SHOOT, SHOOT
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
