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
import frc.robot.lib.team503.util.Util;
import edu.wpi.first.wpilibj.RobotBase;


public class Shooter extends SubsystemBase {
  
    static InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();
    private static Shooter instance = null;
    private int num = 1;
    private int shootMode = 1;//1 means table shoot ,0 means algorithm shoot
    private boolean ShotWhileMove = true;
    private static WPI_TalonFX mShooter;
    private final PeriodicIO periodicIO = new PeriodicIO();
    ShooterControlState currentState = ShooterControlState.STOP;
    private double MoveOffset = 0;
    private boolean readyToShoot = false;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private boolean velocityStabilized = true;
    double targetVelocity ;
    double targetHoodAngle ;
    double targetTurretAngle;

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

    static { //TODO first is meters for distance,second is RPM
        SHOOTER_TUNING.put(new InterpolatingDouble(0.0000), new Vector2(70,3000));//TODO
        SHOOTER_TUNING.put(new InterpolatingDouble(1.9304), new Vector2(55,3500));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.5400), new Vector2(50,4000));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.3020), new Vector2(45,4500));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0640), new Vector2(40,5000));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.8260), new Vector2(35,5500));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0960), new Vector2(30,6000));
        SHOOTER_TUNING.put(new InterpolatingDouble(7.6200), new Vector2(25,6500));
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
        SmartDashboard.putString("Shooter State", currentState.name());   
        SmartDashboard.putNumber("Shooter Mode", shootMode);
        SmartDashboard.putBoolean("Shooter ready",shootIsReady());
        SmartDashboard.putNumber("Shooter Velocity", periodicIO.flywheel_velocity);
        SmartDashboard.putNumber("targetVelocity", targetVelocity);
        SmartDashboard.putNumber("targetHoodAngle", targetHoodAngle);
        SmartDashboard.putNumber("targetTurretAngle", targetTurretAngle);
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
            
        periodicIO.flywheel_velocity = getShooterSpeedRpm();
        periodicIO.flywheel_voltage = mShooter.getMotorOutputVoltage();
        periodicIO.flywheel_current = mShooter.getSupplyCurrent();
        periodicIO.flywheel_temperature = mShooter.getTemperature();

    }

    public void writePeriodicOutputs() {
        if(currentState == ShooterControlState.STOP) {
            mShooter.set(ControlMode.PercentOutput, 0);
            Hood.getInstance().setHoodStop();
        }
        if(currentState == ShooterControlState.INIT) {
            mShooter.set(ControlMode.PercentOutput, 0);
        }
        if(currentState == ShooterControlState.PREPARE_SHOOT){
            double cal_shooterFeedForward = shooterFeedForward.calculate(FalconToMeterSpeed(Constants.kFlywheelIdleVelocity));
            periodicIO.flywheel_demand = RpmToFalcon(Constants.kFlywheelIdleVelocity);
            mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
        }
        if(currentState == ShooterControlState.SHOOT){
            if(isShooterCanShoot()){  
                Blocker.getInstance().turnonballLocker();
            }
        }
    }

    public int getShooterMode(){
        return shootMode;
    }

    public void setFixedShootParams(){
        double targetVelocity = getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE);
        double cal_shooterFeedForward = shooterFeedForward.calculate(targetVelocity);
        periodicIO.flywheel_demand = RpmToFalcon(targetVelocity);
        mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
    }

    public void SetMovingShootParams(){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(LimelightSubsystem.getInstance().getRobotToTargetDistance_Opt().orElse(0.0)));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                        RpmToMeterSpeed(angleAndSpeed.y),                             // Shot Speed
                        angleAndSpeed.x,                             // Hood Angle
                        Turret.getInstance().getAngle() + LimelightSubsystem.getInstance().Get_tx() +SwerveDriveTrain.getInstance().GetHeading_Deg(),// Turret target angle (The same coordinate system with swerve)
                        SwerveDriveTrain.getInstance().GetVxSpeed(), // Swerve speed in X axis (field-oriented)
                        SwerveDriveTrain.getInstance().GetVySpeed());// Swerve speed in Y axis (field-oriented)
        targetVelocity = meterSpeedToRpm(mShotParams[2]);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - SwerveDriveTrain.getInstance().GetHeading_Deg());
        MoveOffset = targetTurretAngle - Turret.getInstance().getAngle();
        double cal_shooterFeedForward = shooterFeedForward.calculate(RpmToMeterSpeed(targetVelocity));
        periodicIO.flywheel_demand = RpmToFalcon(targetVelocity);
        Hood.getInstance().setHoodAngle(targetHoodAngle);
        mShooter.set(ControlMode.Velocity, periodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
    }

    public boolean isShooterCanShoot(){
        boolean result = false;
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(LimelightSubsystem.getInstance().getRobotToTargetDistance_Opt().orElse(0.0)));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                        RpmToMeterSpeed(angleAndSpeed.y),                             // Shot Speed
                        angleAndSpeed.x,                             // Hood Angle
                        Turret.getInstance().getAngle() + SwerveDriveTrain.getInstance().GetHeading_Deg(),// Turret Angle (The same coordinate system with swerve)
                        SwerveDriveTrain.getInstance().GetVxSpeed(), // Swerve speed in X axis (field-oriented)
                        SwerveDriveTrain.getInstance().GetVySpeed());// Swerve speed in Y axis (field-oriented)
        targetVelocity = meterSpeedToRpm(mShotParams[2]);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - SwerveDriveTrain.getInstance().GetHeading_Deg());
        result = (Math.abs(targetVelocity - getShooterSpeedRpm()) < 50) 
                 &&( Math.abs(targetTurretAngle - Turret.getInstance().getAngle()) < 1.0)
                 &&( Math.abs(targetHoodAngle - Hood.getInstance().getHoodAngle()) < 1.0)
                ; 
        return result;
    }

    public void shootOnMoveOrbit(){
        
    }

    public void shootFlywheel(double meterSpeed) {
        // meters per second
        double driveOutput = meterSpeed 
        / (Constants.kFlyWheelWheelDiameter * Math.PI)
        / Constants.kFlyWheelEncoderReductionRatio
        * 2048.0 * 0.1;
        periodicIO.flywheel_demand = driveOutput;
        mShooter.set(ControlMode.Velocity,driveOutput);
    }

    public boolean shootIsReady(){
        return isShooterCanShoot();
    }

    public double getFlywheelVelocity() {
        // meters per second
        double speed = 0;
        double rawunits = 0;
        if (RobotBase.isSimulation()) {
            rawunits = periodicIO.flywheel_demand;
        }else{
            rawunits = mShooter.getSelectedSensorVelocity();
        }
        speed = rawunits / 0.1 / 2048.0 
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
        if (RobotBase.isSimulation()) {
            return (periodicIO.flywheel_demand * 600.0) / 2048.0;
        }else{
            return mShooter.getSelectedSensorVelocity() * 600.0 / 2048.0;
        }

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

    public double RpmToFalcon(double rpm){
        double FalconUnits = MeterSpeedToFalcon(RpmToMeterSpeed(rpm));
        return FalconUnits;
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
            shootMode = 1; //hood can move
          }
          else{
            shootMode = 0; //hood is fixed
          }
          num += 1;
    }

    public void setShootShooter(){
        currentState = ShooterControlState.SHOOT;
    }
    public void setPrepareShooter(){
        currentState = ShooterControlState.PREPARE_SHOOT;
    }

    public void setStopShooter(){
        currentState = ShooterControlState.STOP;
    }

    public void setInitShooter(){
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
        double phi_v_rad = Math.toRadians(phi_v);
        double phi_h_rad = Math.toRadians(phi_h);

        double theta_h_rad = Math.atan2((V*Math.cos(phi_v_rad)*Math.sin(phi_h_rad) - vy) ,
                                    (V*Math.cos(phi_v_rad)*Math.cos(phi_h_rad) - vx));
        double theta_v_rad = Math.atan(1/((V*Math.cos(phi_v_rad)*Math.cos(phi_h_rad) -
                                        vx) / (V*Math.sin(phi_v_rad)*Math.cos(theta_h_rad))));
        double vs = V*Math.sin(phi_v_rad)/Math.sin(theta_v_rad);

        double theta_h = Math.toDegrees(theta_h_rad);
        double theta_v = Math.toDegrees(theta_v_rad);

        double[] myShootParams = new double [3];
        myShootParams[0] = theta_h;
        myShootParams[1] = theta_v;
        myShootParams[2] = vs;

        return myShootParams;
    }

    public double getMoveOffset(){
        return MoveOffset;
    }

    public boolean whetherReadyToShoot(){
        return readyToShoot;
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
