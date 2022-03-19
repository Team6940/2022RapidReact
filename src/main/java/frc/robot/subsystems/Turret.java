package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.team503.util.Util;


public class Turret extends SubsystemBase {

    private static final double maxSoftLimit = 100; //度数  //TODO
    private static final double minSoftLimit = -193;  //TODO
    private static int direction = 0;
    private static Turret instance = null;
    WPI_TalonSRX mTurretMotor;
    PeriodicIO periodicIO = new PeriodicIO();
    private int offset = 1080; //TODO
    private TurretControlState currentState = TurretControlState.ZERO_TURRET;
  
    public Turret() {
        mTurretMotor = new WPI_TalonSRX(Constants.turretID);
        //mTurretMotor.configFactoryDefault();
        mTurretMotor.setInverted(false);
        mTurretMotor.setSensorPhase(false);
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

    public double getAngle() {
        double rawTurretAngle = encUnitsToTurretAngle(periodicIO.position - offset);
        double Angle = Util.boundAngleNeg180to180Degrees(rawTurretAngle);
        return Angle;
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / 4096.0 * 360.0;
    }

    public int degreesToEncUnits(double degrees) {
        return (int) (degrees / 360.0 * 4096.0);
    }

    public double encUnitsToTurretAngle(int encUnits) {
        return Constants.kTurretStartingAngle + encUnitsToDegrees(encUnits);//TODO
    }

    public int turretAngleToEncUnits(double mTurretMotorAngle) {
        return degreesToEncUnits(mTurretMotorAngle - Constants.kTurretStartingAngle);//Todo
    }

    public boolean isSensorConnected() {
        int pulseWidthPeriod = mTurretMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        //if (!connected)
        //    hasEmergency = true;
        return connected;
    }

    public void readPeriodicInputs() {
        periodicIO.position = (int) mTurretMotor.getSelectedSensorPosition(0);
        periodicIO.velocity = (int) mTurretMotor.getSelectedSensorVelocity(0);
        periodicIO.voltage = mTurretMotor.getMotorOutputVoltage();
        periodicIO.current = mTurretMotor.getStatorCurrent();
    }

    public boolean isTurretReady() {
        return LimelightSubsystem.getInstance().isTargetVisible() && (LimelightSubsystem.getInstance().Get_tx() < 1.0);
    }

    public void writePeriodicOutputs() {
        double desiredAngle = 0;
        if (currentState == TurretControlState.ZERO_TURRET) {
            periodicIO.demand = turretAngleToEncUnits(desiredAngle);
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);

        }
        else if(currentState == TurretControlState.VISION_LOCKED) {                
            desiredAngle = getAngle();
            periodicIO.demand = turretAngleToEncUnits(desiredAngle);
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);

        }
        else if (currentState == TurretControlState.VISION_MOVING) {
            desiredAngle =  LimelightSubsystem.getInstance().Get_tx()  + getAngle();
            desiredAngle = Util.boundAngleNeg180to180Degrees(desiredAngle);
            periodicIO.demand = turretAngleToEncUnits(desiredAngle);
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            if (!LimelightSubsystem.getInstance().isTargetVisible()) {
                currentState = TurretControlState.VISION_FINDING;
                if(desiredAngle < 0){
                    direction = 0;
                }
                else{
                    direction = 1;
                }
            }
            if (Math.abs(LimelightSubsystem.getInstance().Get_tx()) < 1.0 && LimelightSubsystem.getInstance().isTargetVisible()) {
                lockOnTarget();
            }

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
            periodicIO.demand = turretAngleToEncUnits(desiredAngle);
            mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            if (LimelightSubsystem.getInstance().isTargetVisible()) {
                startVisionMoving();
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
            //SmartDashboard.putNumber("Turret Current", periodicIO.current);
            SmartDashboard.putNumber("Turret Voltage", periodicIO.voltage);
            SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
            SmartDashboard.putNumber("Turret Demand", periodicIO.demand);
            //SmartDashboard.putNumber("Turret Arb Demand", -512 * OI.getDriverRightXValue());
            SmartDashboard.putNumber("Turret Pulse Width Position",
                    mTurretMotor.getSensorCollection().getPulseWidthPosition());
            SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Turret Error", encUnitsToDegrees(mTurretMotor.getClosedLoopError(0)));
            if (mTurretMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber("Turret Setpoint", mTurretMotor.getClosedLoopTarget(0));
        }
    }

    public void startVisionFind(){
        currentState = TurretControlState.VISION_FINDING;
    }

    public void VisionLock(){
        currentState = TurretControlState.VISION_LOCKED;
    }


    public enum TurretControlState {
        ZERO_TURRET, VISION_LOCKED, VISION_MOVING, VISION_FINDING
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
