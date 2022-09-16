package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
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
    private static Shooter instance = null;//设定intake
    private static WPI_TalonFX mShooterLeft;//左shooter电机
    private static WPI_TalonFX mShooterRght;//右shooter电机
    private final ShooterPeriodicIO ShooterPeriodicIO = new ShooterPeriodicIO();//???
    ShooterControlState shootState = ShooterControlState.STOP;//shooter状态
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private double desiredShooterSpeed = 0;

    /**
     * We use FeedForward and a little P gains for Shooter
     */
    SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);//???

    // for hood
    private WPI_TalonSRX mHoodmotor;//hood一号电机
    private WPI_TalonFX mHoodmotor2;//hood二号电机
    private int offset = 622;//TODO encoder的偏移量
    HoodPeriodicIO HoodPeriodicIO = new HoodPeriodicIO();//新定义一个shooter状态类
    private double desiredHoodAngle;//目标hood角度
    HoodControlState hoodstate = HoodControlState.HOME;//hood状态，默认归零
  
    // for blocker
    private WPI_TalonFX blockerMotor;//blocker电机

    public Shooter() {//shooter构造函数
        configShooter();//设定shooter
        configHood();//设定hood
        configBlocker();//设定blocker
    }

    public static Shooter getInstance() {//返回当前正在使用的shooter类
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    private void configShooter() {//设定shooter
        TalonFXConfiguration lMasterConfig = new TalonFXConfiguration();
        //各项pid参数，但是lmasterconfig是啥
        lMasterConfig.slot0.kP = 0.0000005;//TODO
        lMasterConfig.slot0.kI = 0;
        lMasterConfig.slot0.kD = 0;
        lMasterConfig.slot0.kF = 0.057;
        lMasterConfig.peakOutputForward = 0.8;
        lMasterConfig.peakOutputReverse = 0.0;
        //shooter电机参数设定
        mShooterLeft = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);//设定shooter电机
        mShooterLeft.setInverted(true);//TODO 电机是否反转
        //mShooterLeft.configAllSettings(lMasterConfig);//将pid参数注入到电机中
        mShooterLeft.setNeutralMode(NeutralMode.Coast);//???
        mShooterLeft.config_kP(0, 0.0000005);
        mShooterLeft.config_kI(0, 0);
        mShooterLeft.config_kD(0, 0);
        mShooterLeft.config_kF(0, 0.057);
        mShooterLeft.configPeakOutputForward(1);
        mShooterLeft.configPeakOutputReverse(-1);
        mShooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);//设定反馈传感器???
        mShooterLeft.configVoltageCompSaturation(12);
        mShooterLeft.enableVoltageCompensation(true);
        mShooterLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        //右电机，同上
        mShooterRght = new WPI_TalonFX(Constants.SHOOT_R_MASTER_ID);
        mShooterRght.setInverted(false);//TODO
        mShooterRght.follow(mShooterLeft);
        //mShooterRght.configAllSettings(lMasterConfig);
        mShooterRght.setNeutralMode(NeutralMode.Coast);
        mShooterRght.config_kP(0, 0.0000005);//0.0000005
        mShooterRght.config_kI(0, 0);
        mShooterRght.config_kD(0, 0);
        mShooterRght.config_kF(0, 0.05);//0.05
        mShooterRght.configPeakOutputForward(1.0);
        mShooterRght.configPeakOutputReverse(-1.0);
        mShooterRght.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mShooterRght.configVoltageCompSaturation(12);
        mShooterRght.enableVoltageCompensation(true);
        mShooterRght.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    }

    private void configHood(){//设定hood参数
        //hood电机参数
        mHoodmotor = new WPI_TalonSRX(Constants.HoodMotorPort);
        mHoodmotor2 = new WPI_TalonFX(Constants.HoodMotorPort + 5);
        //设定反转
        mHoodmotor.setInverted(false);
        mHoodmotor.setSensorPhase(false);
        mHoodmotor.setNeutralMode(NeutralMode.Brake);//???
        //设定pid
        mHoodmotor.selectProfileSlot(0, 0);//TODO
        mHoodmotor.config_kP(0, 0.5, 10);
        mHoodmotor.config_kI(0, 0.0, 10);
        mHoodmotor.config_kD(0, 0.0, 10);
        mHoodmotor.config_kF(0, 0.00, 10);
        mHoodmotor.config_IntegralZone(0, 0, 10);
        mHoodmotor.configMotionCruiseVelocity(600, 10);//???
        mHoodmotor.configMotionAcceleration(1200, 10);//???
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
        double cal_shooterFeedForward = shooterFeedForward.calculate(Conversions.RPMToMPS(desiredShooterSpeed, Constants.kFlyWheelCircumference));
        ShooterPeriodicIO.flywheel_demand = Conversions.RPMToFalcon(desiredShooterSpeed,Constants.kFlyWheelEncoderReductionRatio);
        mShooterLeft.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
        //mShooterLeft.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand);
        //mShooterRght.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand);
        
        if(shootState == ShooterControlState.SHOOT){
            //if(VisionManager.getInstance().isShooterCanShoot()){  //TODO  need to debug what is can shoot condition
            /*if(AimManager.getInstance().CanShot()){
                setFiring(true);
            }*/
        }
    }

    public synchronized double getShooterSpeedRpm() {//以rpm的单位获取shooter转速
        if (RobotBase.isSimulation()) {
            return Conversions.falconToRPM(ShooterPeriodicIO.flywheel_demand, Constants.kFlyWheelEncoderReductionRatio);
        }else{
            return Conversions.falconToRPM(mShooterLeft.getSelectedSensorVelocity(), Constants.kFlyWheelEncoderReductionRatio);
        }

    }
    public synchronized double getDesiredShooterSpeedRpm() {
        return desiredShooterSpeed;
    }

    public void setShooterToShoot(){//将shooter状态改为射球状态
        shootState = ShooterControlState.SHOOT;
    }
    public void setShooterToPrepare(){//将shooter状态改为准备状态
        shootState = ShooterControlState.PREPARE_SHOOT;
    }

    public void setShooterToStop(){//将shooter状态改为停止状态
        shootState = ShooterControlState.STOP;
    }

    public void setShooterToMannulShoot(){//???
        shootState = ShooterControlState.MANNUL_SHOOT;
    }
    
    public enum ShooterControlState {
        STOP, PREPARE_SHOOT, SHOOT,MANNUL_SHOOT
    }

    public class ShooterPeriodicIO {//shooter某一时刻的状态类
        //INPUT
        public double timestamp;//???
        
        public double flywheel_velocity;//飞轮速度
        public double flywheel_voltage;//飞轮电压
        public double slave_voltage;//跟随主电机的电机的电压
        public double flywheel_current;//???
        public double flywheel_temperature;//飞轮温度

        //OUTPUTS
        public double flywheel_demand;//???
        public double kickerDemand;//???
    }

    /**
     * Sets Speed of Shooter FlywheelWheel.
     *
     * @param RPM Desired Speed in RPM.
     */
    public void setShooterSpeed(double RPM) {//设定shooter转速
        this.desiredShooterSpeed = RPM;
        if (desiredShooterSpeed == 0) {
            shootState = ShooterControlState.STOP;
        }else{
            shootState = ShooterControlState.SHOOT;
        }
    }

    // for hood
    public enum HoodControlState {//hood控制状态
        HOME, ON, STOP
    }

    public HoodControlState getHoodState() {//获取hood状态
        return hoodstate;
    }

    public void setHoodAngle(double targetAngle){//设定hood角度（以实际角度为单位
        // Preform Bounds checking between MAX and MIN range
        if (targetAngle < Constants.HOOD_MIN_ANGLE) {//作上下约束
            targetAngle = Constants.HOOD_MIN_ANGLE;
        }

        if (targetAngle > Constants.HOOD_MAX_ANGLE) {
            targetAngle = Constants.HOOD_MAX_ANGLE;
        }
        desiredHoodAngle = targetAngle;   //设定hood目标角度
        hoodstate = HoodControlState.ON;//开启hood
    }
    
    public void setHoodToStop(){//设定Hood状态为stop
        hoodstate = HoodControlState.STOP;
    }

    public void setHoodToHome(){//设定Hood状态为home
        hoodstate = HoodControlState.HOME;
    }

    public void setHoodToOn(){//设定hood状态为on
        hoodstate = HoodControlState.ON;
    }
    
    public double getHoodAngle(){//获取hood当前角度
        if (RobotBase.isSimulation()){
          return Conversions.talonToDegrees(HoodPeriodicIO.position - offset, Constants.HOOD_GEAR_RATIO);
        }else{
          return Conversions.talonToDegrees((int) mHoodmotor.getSelectedSensorPosition(0)- offset, Constants.HOOD_GEAR_RATIO);
        }
        
    }
    public double getDesiredHoodAngle(){//获取Hood目标角度
        return desiredHoodAngle;
    }

    public void HoodWritePeriodicOutputs(){//重复执行的函数
        if(hoodstate == HoodControlState.HOME){//如果hood的状态为home
            desiredHoodAngle = Constants.HOOD_HOME_ANGLE;//将hood电机转到home角度
        }else if(hoodstate == HoodControlState.STOP){//如果是stop，那么保持电机角度
            desiredHoodAngle = getHoodAngle();
        }else if(hoodstate == HoodControlState.ON){
            ;
        }
        double targetPos = Conversions.degreesToTalon(desiredHoodAngle, Constants.HOOD_GEAR_RATIO) + offset;//计算目标电机位置（以Unit为单位
        //double targetPosFalcon = Conversions.degreesToFalcon(desiredHoodAngle, Constants.HOOD_GEAR_RATIO);
        HoodPeriodicIO.demand = (int) targetPos;//设定电机目标位置
        mHoodmotor.set(ControlMode.MotionMagic, targetPos);//输出电机位置
        //mHoodmotor2.set(ControlMode.MotionMagic, targetPosFalcon);
    }

    public void HoodOutputTelemetry(){
        ;
    }
    
    public void HoodReadPeriodicInputs() {
        if (RobotBase.isSimulation())
        {
            HoodPeriodicIO.position = (int)HoodPeriodicIO.demand;//获取hood位置
        }else{
            HoodPeriodicIO.position = (int) mHoodmotor.getSelectedSensorPosition(0);//h
        }
    }
    
    public static class HoodPeriodicIO {

        // Inputs
        public int position;
        // Outputs
        public double demand;
    }


    // for blocker

    public enum BlockerControlState {//blocker状态，开和关
        BALLLOCKER_ON,
        BALLLOCKER_OFF
    }

    private BlockerControlState blockerState = BlockerControlState.BALLLOCKER_OFF;//blocker默认状态是关

    /**
     * Turns the shooter blocker on/off
     *
     * @param shoot true to turn on, false to turn off
     */
    public void setFiring(boolean shoot) {//设定开火
        blockerState = shoot ? BlockerControlState.BALLLOCKER_ON : BlockerControlState.BALLLOCKER_OFF;
    }

    /**
     * Gets the state of the Feeder Wheel.
     * <p>
     * Can either be ON or OFF.
     */
    public BlockerControlState getBlockerState(){//获取blocker状态
        return blockerState;
    }

    public void BlockerWritePeriodicOutputs() {
        if (blockerState == BlockerControlState.BALLLOCKER_ON) {//blocker状态
            blockerMotor.set(ControlMode.PercentOutput, BlockerConstants.kFireSpeed);//如果blocker状态为on，那么按照开火速度旋转blockermotor
        } else {
            blockerMotor.set(ControlMode.PercentOutput, BlockerConstants.kStopSpeed);//反之让blocker停止
        }
    }

    @Override
    public void periodic() {//反复执行的函数都在这里
        HoodWritePeriodicOutputs();
        HoodReadPeriodicInputs();
        HoodOutputTelemetry();
        ShooterWritePeriodicOutputs();
        ShooterReadPeriodicInputs();
        ShooterOutputTelemetry();
        BlockerWritePeriodicOutputs();
    }
}
