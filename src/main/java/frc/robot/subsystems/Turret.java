package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.team503.util.Util;
import frc.robot.lib.team1678.math.Conversions;


public class Turret extends SubsystemBase {

    private static final double maxSoftLimit = 100; //度数  //TODO
    private static final double minSoftLimit = -193;  //TODO
    private static int direction = 0;
    private static Turret instance = null;
    WPI_TalonSRX mTurretMotor;
    TalonSRXSimCollection mTurretSensorSim ;
    PeriodicIO periodicIO = new PeriodicIO();
    private int offset = 1080; //TODO
    private TurretControlState currentState = TurretControlState.ZERO_TURRET;
  
    public Turret() {
        mTurretMotor = new WPI_TalonSRX(Constants.turretID);
        if (RobotBase.isSimulation()){
            mTurretSensorSim = mTurretMotor.getSimCollection();
        }
        //mTurretMotor.configFactoryDefault();
        mTurretMotor.setInverted(false);
        mTurretMotor.setSensorPhase(false);//TODO
        setBrakeMode(false);

        configurationOne();
        mTurretMotor.configForwardSoftLimitThreshold(turretAngleToEncUnits(maxSoftLimit), 10); //TODO
        mTurretMotor.configReverseSoftLimitThreshold(turretAngleToEncUnits(minSoftLimit), 10); //TODO
        mTurretMotor.configForwardSoftLimitEnable(true, 10);
        mTurretMotor.configReverseSoftLimitEnable(true, 10);

        mTurretMotor.configPeakOutputForward(0.50, 10);//TODO
        mTurretMotor.configPeakOutputReverse(-0.50, 10);//TODO

        mTurretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    }

    public static Turret getInstance() {
        if (instance == null){
            instance = new Turret();
        }
        return instance;
    }

    private void configurationOne() {
        mTurretMotor.config_kP(0, 3.66, 10);//TODO
        mTurretMotor.config_kI(0, 0, 10);
        mTurretMotor.config_kD(0, 10.0, 10);//TODO
        mTurretMotor.config_kF(0, 0.00, 10);
        mTurretMotor.config_IntegralZone(0, 0, 10);
        mTurretMotor.configMotionCruiseVelocity(600, 10);
        mTurretMotor.configMotionAcceleration(1200, 10);
        mTurretMotor.configVoltageCompSaturation(12);
        mTurretMotor.enableVoltageCompensation(true);
        // mTurretMotor.configMotionSCurveStrength(6);
    }

    public void setBrakeMode(boolean brake) {
        mTurretMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public TurretControlState getTurretState() {
        return currentState;
    }

    public void setTurretState(TurretControlState state) {
        currentState = state;
    }

    public void ZeroTurret() {
        currentState = TurretControlState.ZERO_TURRET;
    }

    public void startVisionMoving() {
        currentState = TurretControlState.VISION_MOVING;
    }

    public void lockOnTarget() {
        currentState = TurretControlState.VISION_LOCKED;
    }

    public boolean isVisionFinding(){
        return (currentState == TurretControlState.VISION_FINDING);
    }

    public boolean isVisionMoving(){
        return (currentState == TurretControlState.VISION_MOVING);
    }

    public boolean isVisionLocked(){
        return (currentState == TurretControlState.VISION_LOCKED);
    }

    public boolean valueIsRange(double value, double minRange,double maxRange){
        return Math.max(minRange,value) == Math.min(value,maxRange);
    }

    public boolean isVisionGoodRange(double angle){    
        return ( valueIsRange(angle,minSoftLimit,maxSoftLimit)
                && LimelightSubsystem.getInstance().isTargetVisible()
                );
    }

    public boolean isStop(){
        return (currentState == TurretControlState.STOP);
    }

    public void startVisionFinding(){
        currentState = TurretControlState.VISION_FINDING;
    }

    public void Stop(){
        currentState = TurretControlState.STOP;
    }

    public double getAngle() {
        double rawTurretAngle = encUnitsToTurretAngle(periodicIO.position - offset);
        double Angle = Util.boundAngleNeg180to180Degrees(rawTurretAngle);
        return Angle;
    }

    public double encUnitsToTurretAngle(int encUnits) {
        return Constants.kTurretStartingAngle + Conversions.talonToDegrees((double)encUnits,Constants.TURRET_GEAR_RATIO);//TODO
    }

    public int turretAngleToEncUnits(double mTurretMotorAngle) {
        return (int)Conversions.degreesToTalon((mTurretMotorAngle - Constants.kTurretStartingAngle),Constants.TURRET_GEAR_RATIO);//Todo
    }

    public void readPeriodicInputs() {
        if (RobotBase.isSimulation())
        {
            periodicIO.position = (int)periodicIO.demand;
        }else{
            periodicIO.position = (int) mTurretMotor.getSelectedSensorPosition(0);  
            periodicIO.velocity = (int) mTurretMotor.getSelectedSensorVelocity(0);
        }
        periodicIO.voltage = mTurretMotor.getMotorOutputVoltage();
        periodicIO.current = mTurretMotor.getStatorCurrent();
    }

    public boolean isTargetReady(){
        return (Math.abs(LimelightSubsystem.getInstance().Get_tx()) < 1.0 
                && LimelightSubsystem.getInstance().isTargetVisible()
                );
    }

    public void writePeriodicOutputs() {
        double desiredAngle = 0;
        if (currentState == TurretControlState.ZERO_TURRET) {
            periodicIO.demand = turretAngleToEncUnits(desiredAngle) + offset;
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
        }else if(currentState == TurretControlState.STOP){
            desiredAngle = getAngle();
            periodicIO.demand = turretAngleToEncUnits(desiredAngle) + offset;
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
        }else if (currentState == TurretControlState.VISION_FINDING) {
            desiredAngle = getAngle();
            if(direction == 0 ){
                desiredAngle -= Constants.kTurretStep;
                desiredAngle = Math.max(desiredAngle, minSoftLimit);
                if(desiredAngle <= minSoftLimit){
                    direction = 1;
                }
            }else{
                desiredAngle += Constants.kTurretStep;
                desiredAngle = Math.min(desiredAngle, maxSoftLimit);
                if(desiredAngle >= maxSoftLimit){
                    direction = 0;
                }
            }
            periodicIO.demand = turretAngleToEncUnits(desiredAngle) + offset;
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            if (isVisionGoodRange(LimelightSubsystem.getInstance().Get_tx()  + getAngle())) {
                startVisionMoving();
            }         
        }else if (currentState == TurretControlState.VISION_MOVING) {
            desiredAngle =  LimelightSubsystem.getInstance().Get_tx()  + getAngle();
            desiredAngle = Util.boundAngleNeg180to180Degrees(desiredAngle);
            periodicIO.demand = turretAngleToEncUnits(desiredAngle) + offset;
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            if (isTargetReady()) {
                lockOnTarget();
            }else if (!isVisionGoodRange(LimelightSubsystem.getInstance().Get_tx()  + getAngle())) {
                startVisionFinding();
                if(desiredAngle < 0){
                    direction = 0;
                }
                else{
                    direction = 1;
                }
            }
        }else if(currentState == TurretControlState.VISION_LOCKED) {                
            desiredAngle = getAngle();
            periodicIO.demand = turretAngleToEncUnits(desiredAngle) + offset;
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            if (!isTargetReady()) {
                startVisionFinding();
                if(desiredAngle < 0){
                    direction = 0;
                }
                else{
                    direction = 1;
                }
            }
        }
    }

    public void stop() {
        ZeroTurret();
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Turret State", getTurretState().name());   
        SmartDashboard.putNumber("Turret Angle", getAngle());

        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
            SmartDashboard.putNumber("Turret Demand", periodicIO.demand);
            SmartDashboard.putNumber("Turret Error", Conversions.talonToDegrees(mTurretMotor.getClosedLoopError(0), Constants.TURRET_GEAR_RATIO));
            if (mTurretMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber("Turret Setpoint", mTurretMotor.getClosedLoopTarget(0));
        }
    }

    public enum TurretControlState {
        ZERO_TURRET, VISION_LOCKED, VISION_MOVING, VISION_FINDING, STOP
    }

    public static class PeriodicIO {
        // Inputs
        public int position;
        public int velocity;
        public double voltage;
        public double current;

        // Outputs
        public double demand;
    }
}
