package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.BlockerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.team1678.math.Conversions;
import edu.wpi.first.wpilibj.RobotBase;


public class Shooter extends SubsystemBase {
    private static Shooter instance = null;
    private static WPI_TalonFX mShooterLeft;
    private static WPI_TalonFX mShooterRght;
    private final ShooterPeriodicIO ShooterPeriodicIO = new ShooterPeriodicIO();
    ShooterControlState shootState = ShooterControlState.STOP;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private double desiredShooterSpeed = 0;

    /**
     * We use FeedForward and a little P gains for Shooter
     */
    SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);

    // for hood
    private WPI_TalonSRX mHoodmotor;
    private WPI_TalonFX mHoodmotor2;
    private int offset = 622;//TODO
    HoodPeriodicIO HoodPeriodicIO = new HoodPeriodicIO();
    private double desiredHoodAngle;
    HoodControlState hoodstate = HoodControlState.HOME;
  
    // for blocker
    private WPI_TalonFX blockerMotor;

    public Shooter() {
        configShooter();
        configHood();
        configBlocker();
    }

    public static Shooter getInstance() {
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    private void configShooter() {
        TalonFXConfiguration lMasterConfig = new TalonFXConfiguration();

        lMasterConfig.slot0.kP = 0.05;//TODO
        lMasterConfig.slot0.kI = 0;
        lMasterConfig.slot0.kD = 0;
        lMasterConfig.slot0.kF = 0.115;
        lMasterConfig.peakOutputForward = 0.8;
        lMasterConfig.peakOutputReverse = 0.0;
        mShooterLeft = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);
        mShooterLeft.setInverted(true);//TODO
        mShooterLeft.configAllSettings(lMasterConfig);
        mShooterLeft.setNeutralMode(NeutralMode.Coast);
        mShooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mShooterLeft.configVoltageCompSaturation(12);
        mShooterLeft.enableVoltageCompensation(true);

        mShooterRght = new WPI_TalonFX(Constants.SHOOT_R_MASTER_ID);
        mShooterRght.setInverted(false);//TODO
        mShooterRght.follow(mShooterLeft);
        mShooterRght.configAllSettings(lMasterConfig);
        mShooterRght.setNeutralMode(NeutralMode.Coast);
        mShooterRght.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mShooterRght.configVoltageCompSaturation(12);
        mShooterRght.enableVoltageCompensation(true); 
    }

    private void configHood(){
        mHoodmotor = new WPI_TalonSRX(Constants.HoodMotorPort);
        mHoodmotor2 = new WPI_TalonFX(Constants.HoodMotorPort + 5);

        mHoodmotor.setInverted(false);
        mHoodmotor.setSensorPhase(false);
        mHoodmotor.setNeutralMode(NeutralMode.Brake);
    
        mHoodmotor.selectProfileSlot(0, 0);//TODO
        mHoodmotor.config_kP(0, 0.5, 10);
        mHoodmotor.config_kI(0, 0.0, 10);
        mHoodmotor.config_kD(0, 0.0, 10);
        mHoodmotor.config_kF(0, 0.00, 10);
        mHoodmotor.config_IntegralZone(0, 0, 10);
        mHoodmotor.configMotionCruiseVelocity(600, 10);
        mHoodmotor.configMotionAcceleration(1200, 10);
        // mHoodmotor.configMotionSCurveStrength(6);

        mHoodmotor.configForwardSoftLimitThreshold(Conversions.degreesToTalon(Constants.HOOD_MAX_ANGLE, Constants.HOOD_GEAR_RATIO) + offset, 10); //TODO
        mHoodmotor.configReverseSoftLimitThreshold(Conversions.degreesToTalon(Constants.HOOD_MIN_ANGLE, Constants.HOOD_GEAR_RATIO), 10); //TODO
        mHoodmotor.configForwardSoftLimitEnable(true, 10);
        mHoodmotor.configReverseSoftLimitEnable(true, 10);
    
        mHoodmotor.configVoltageCompSaturation(12);
        mHoodmotor.enableVoltageCompensation(true);
    
        mHoodmotor.configPeakOutputForward(0.50, 10);//TODO
        mHoodmotor.configPeakOutputReverse(-0.50, 10);//TODO

        mHoodmotor2.config_kF(0, 0);
        mHoodmotor2.config_kP(0, 0);//TODO
        mHoodmotor2.config_kI(0, 0);
        mHoodmotor2.config_kD(0, 0);
        mHoodmotor2.config_IntegralZone(0, 0);
        mHoodmotor2.configPeakOutputForward(1);
        mHoodmotor2.configPeakOutputReverse(-1);
        mHoodmotor2.setNeutralMode(NeutralMode.Brake);
        mHoodmotor2.configMotionAcceleration(3000);
        mHoodmotor2.configMotionCruiseVelocity(3000);

        mHoodmotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        mHoodmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    }

    private void configBlocker(){
        blockerMotor = new WPI_TalonFX(BlockerConstants.kBlockerID);
    }

    public ShooterControlState getShooterState() {
        return shootState;
    }

    public void ShooterOutputTelemetry() {
        ;
    }

    public void ShooterReadPeriodicInputs() {
        ShooterPeriodicIO.timestamp = Timer.getFPGATimestamp();
            
        ShooterPeriodicIO.flywheel_velocity = getShooterSpeedRpm();
        ShooterPeriodicIO.flywheel_voltage = mShooterLeft.getMotorOutputVoltage();
        ShooterPeriodicIO.flywheel_current = mShooterLeft.getSupplyCurrent();
        ShooterPeriodicIO.flywheel_temperature = mShooterLeft.getTemperature();

    }

    public void ShooterWritePeriodicOutputs() {
        if(shootState == ShooterControlState.STOP) {
            desiredShooterSpeed = 0;
            //setHoodToStop();
        }
        if(shootState == ShooterControlState.PREPARE_SHOOT){
            desiredShooterSpeed = Constants.kFlywheelIdleVelocity;
        }

        if(shootState == ShooterControlState.MANNUL_SHOOT){
            ;   
        }
        //double cal_shooterFeedForward = shooterFeedForward.calculate(Conversions.RPMToMPS(desiredShooterSpeed, Constants.kFlyWheelCircumference));
        ShooterPeriodicIO.flywheel_demand = Conversions.RPMToFalcon(desiredShooterSpeed,Constants.kFlyWheelEncoderReductionRatio);
        //mShooterLeft.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
        mShooterLeft.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand);
        //mShooterRght.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand);
        
        if(shootState == ShooterControlState.SHOOT){
            //if(VisionManager.getInstance().isShooterCanShoot()){  //TODO  need to debug what is can shoot condition
            /*if(AimManager.getInstance().CanShot()){
                setFiring(true);
            }*/
        }
    }

    public synchronized double getShooterSpeedRpm() {
        if (RobotBase.isSimulation()) {
            return Conversions.falconToRPM(ShooterPeriodicIO.flywheel_demand, Constants.kFlyWheelEncoderReductionRatio);
        }else{
            return Conversions.falconToRPM(mShooterLeft.getSelectedSensorVelocity(), Constants.kFlyWheelEncoderReductionRatio);
        }

    }

    public void setShooterToShoot(){
        shootState = ShooterControlState.SHOOT;
    }
    public void setShooterToPrepare(){
        shootState = ShooterControlState.PREPARE_SHOOT;
    }

    public void setShooterToStop(){
        shootState = ShooterControlState.STOP;
    }

    public void setShooterToMannulShoot(){
        shootState = ShooterControlState.MANNUL_SHOOT;
    }
    
    public enum ShooterControlState {
        STOP, PREPARE_SHOOT, SHOOT,MANNUL_SHOOT
    }

    public class ShooterPeriodicIO {
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

    /**
     * Sets Speed of Shooter FlywheelWheel.
     *
     * @param shooterSpeedRPM Desired Speed in RPM.
     */
    public void setShooterSpeed(double shooterSpeedRPM) {
        this.desiredShooterSpeed = shooterSpeedRPM;
        if (desiredShooterSpeed == 0) {
            shootState = ShooterControlState.STOP;
        }else{
            shootState = ShooterControlState.SHOOT;
        }
    }

    // for hood
    public enum HoodControlState {
        HOME, ON, STOP
    }

    public HoodControlState getHoodState() {
        return hoodstate;
    }

    public void setHoodAngle(double targetAngle){
        // Preform Bounds checking between MAX and MIN range
        if (targetAngle < Constants.HOOD_MIN_ANGLE) {
            targetAngle = Constants.HOOD_MIN_ANGLE;
        }

        if (targetAngle > Constants.HOOD_MAX_ANGLE) {
            targetAngle = Constants.HOOD_MAX_ANGLE;
        }
        desiredHoodAngle = targetAngle;   
        hoodstate = HoodControlState.ON;
    }
    
    public void setHoodToStop(){
        hoodstate = HoodControlState.STOP;
    }

    public void setHoodToHome(){
        hoodstate = HoodControlState.HOME;
    }

    public void setHoodToOn(){
        hoodstate = HoodControlState.ON;
    }
    
    public double getHoodAngle(){
        if (RobotBase.isSimulation()){
          return Conversions.talonToDegrees(HoodPeriodicIO.position - offset, Constants.HOOD_GEAR_RATIO);
        }else{
          return Conversions.talonToDegrees((int) mHoodmotor.getSelectedSensorPosition(0)- offset, Constants.HOOD_GEAR_RATIO);
        }
        
    }

    public void HoodWritePeriodicOutputs(){
        if(hoodstate == HoodControlState.HOME){
            desiredHoodAngle = Constants.HOOD_HOME_ANGLE;
        }else if(hoodstate == HoodControlState.STOP){
            desiredHoodAngle = getHoodAngle();
        }else if(hoodstate == HoodControlState.ON){
            ;
        }
        double targetPos = Conversions.degreesToTalon(desiredHoodAngle, Constants.HOOD_GEAR_RATIO) + offset;
        //double targetPosFalcon = Conversions.degreesToFalcon(desiredHoodAngle, Constants.HOOD_GEAR_RATIO);
        HoodPeriodicIO.demand = (int) targetPos;
        mHoodmotor.set(ControlMode.MotionMagic, targetPos);
        //mHoodmotor2.set(ControlMode.MotionMagic, targetPosFalcon);
    }

    public void HoodOutputTelemetry(){
        ;
    }
    
    public void HoodReadPeriodicInputs() {
        if (RobotBase.isSimulation())
        {
            HoodPeriodicIO.position = (int)HoodPeriodicIO.demand;
        }else{
            HoodPeriodicIO.position = (int) mHoodmotor.getSelectedSensorPosition(0);  
        }
    }
    
    public static class HoodPeriodicIO {

        // Inputs
        public int position;
        // Outputs
        public double demand;
    }


    // for blocker

    public enum BlockerControlState {
        BALLLOCKER_ON,
        BALLLOCKER_OFF
    }

    private BlockerControlState blockerState = BlockerControlState.BALLLOCKER_OFF;

    /**
     * Turns the shooter blocker on/off
     *
     * @param shoot true to turn on, false to turn off
     */
    public void setFiring(boolean shoot) {
        blockerState = shoot ? BlockerControlState.BALLLOCKER_ON : BlockerControlState.BALLLOCKER_OFF;
    }

    /**
     * Gets the state of the Feeder Wheel.
     * <p>
     * Can either be ON or OFF.
     */
    public BlockerControlState getBlockerState(){
        return blockerState;
    }

    public void BlockerWritePeriodicOutputs() {
        if (blockerState == BlockerControlState.BALLLOCKER_ON) {
            blockerMotor.set(ControlMode.PercentOutput, BlockerConstants.kFireSpeed);
        } else {
            blockerMotor.set(ControlMode.PercentOutput, BlockerConstants.kStopSpeed);
        }
    }

    @Override
    public void periodic() {
        HoodWritePeriodicOutputs();
        HoodReadPeriodicInputs();
        HoodOutputTelemetry();
        ShooterWritePeriodicOutputs();
        ShooterReadPeriodicInputs();
        ShooterOutputTelemetry();
        BlockerWritePeriodicOutputs();
    }
}
