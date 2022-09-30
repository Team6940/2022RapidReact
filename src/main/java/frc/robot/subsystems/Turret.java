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
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.lib.team1678.math.Conversions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.team1706.MathUtils;



public class Turret extends SubsystemBase {
    private static Turret instance = null;
    WPI_TalonSRX mTurretMotor;
    TalonSRXSimCollection mTurretSensorSim;
    PeriodicIO periodicIO = new PeriodicIO();
    private int offset = 675;
    private TurretControlState currentState = TurretControlState.STOP;
    private double desiredTurretAngle = 0;
    private boolean m_trackTarget = false;
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();

    public Turret() {
        mTurretMotor = new WPI_TalonSRX(TurretConstants.turretID);
        // mTurretMotor.configFactoryDefault();
        mTurretMotor.setInverted(false);
        mTurretMotor.setSensorPhase(false);
        setBrakeMode(false);

        configurationOne();
        mTurretMotor.configForwardSoftLimitThreshold(turretAngleToEncUnits(TurretConstants.TurretMaxSoftLimitAngle)+offset, 10); // TODO
        mTurretMotor.configReverseSoftLimitThreshold(turretAngleToEncUnits(TurretConstants.TurretMinSoftLimitAngle)+offset, 10); // TODO
        mTurretMotor.configForwardSoftLimitEnable(true, 10);
        mTurretMotor.configReverseSoftLimitEnable(true, 10);

        mTurretMotor.configPeakOutputForward(0.50, 10);// TODO
        mTurretMotor.configPeakOutputReverse(-0.50, 10);// TODO

        mTurretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    }

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private void configurationOne() {
        mTurretMotor.config_kP(0, 0.5, 10);// TODO
        mTurretMotor.config_kI(0, 0, 10);
        mTurretMotor.config_kD(0, 5, 10);// TODO
        mTurretMotor.config_kF(0, 0.00, 10);
        mTurretMotor.config_IntegralZone(0, 0, 10);
        //mTurretMotor.configMotionCruiseVelocity(600, 10);
        //mTurretMotor.configMotionAcceleration(1200, 10);
        mTurretMotor.configVoltageCompSaturation(10);
        mTurretMotor.enableVoltageCompensation(true);
        // mTurretMotor.configMotionSCurveStrength(6);
    }

    public void setBrakeMode(boolean brake) {
        mTurretMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public TurretControlState getTurretState() {
        return currentState;
    }

    public void HomeTurret() {
        currentState = TurretControlState.HOME;
    }
    public void setTurretState(TurretControlState state) {
        currentState = state;
    }

    public boolean isStop() {
        return (currentState == TurretControlState.STOP);
    }

    public boolean isOn() {
        return (currentState == TurretControlState.ON);
    }    

    public void On(boolean on) {
        currentState = on ? TurretControlState.ON : TurretControlState.STOP;
    }

    public double getTurretAngleDeg() {
        double rawTurretAngle = 0;
        if (RobotBase.isSimulation()){
            rawTurretAngle = encUnitsToTurretAngle((int)periodicIO.position - offset);
        }else{
            rawTurretAngle = encUnitsToTurretAngle((int)mTurretMotor.getSelectedSensorPosition(0) - offset);
        }
        
        return rawTurretAngle;
        //double Angle = Util.boundAngleNeg180to180Degrees(rawTurretAngle);
        //return Angle;
    }

    public double getAngleRad(){
        return Math.toRadians(getTurretAngleDeg());
    }

    // here angle is  degrees
    public void setTurretAngle(double angle) {
        if (angle <= TurretConstants.TurretMinSoftLimitAngle) {
            desiredTurretAngle = TurretConstants.TurretMinSoftLimitAngle;
        }else if (angle >= TurretConstants.TurretMaxSoftLimitAngle) {
            desiredTurretAngle = TurretConstants.TurretMaxSoftLimitAngle;
        }else{
            desiredTurretAngle = angle;
        }
    }

    public double encUnitsToTurretAngle(int encUnits) {
        return Conversions.talonToDegrees((double) encUnits, TurretConstants.TURRET_GEAR_RATIO);// TODO
    }

    public int turretAngleToEncUnits(double mTurretMotorAngle) {
        return (int) Conversions.degreesToTalon(mTurretMotorAngle ,TurretConstants.TURRET_GEAR_RATIO);// Todo
    }

    public void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            periodicIO.position = periodicIO.demand;
        } else {
            periodicIO.position = mTurretMotor.getSelectedSensorPosition(0);
            periodicIO.velocity = (int) mTurretMotor.getSelectedSensorVelocity(0);
        }
        periodicIO.voltage = mTurretMotor.getMotorOutputVoltage();
        periodicIO.current = mTurretMotor.getStatorCurrent();
    }

    public void writePeriodicOutputs() {
        if (currentState == TurretControlState.HOME) { 
            desiredTurretAngle = TurretConstants.kTurretStartingAngle;   //TODO  what is home angle?
        } 
        if (currentState == TurretControlState.STOP) {
            desiredTurretAngle = getTurretAngleDeg();
        } 
        if (currentState == TurretControlState.ON) {
            ;
        }

        periodicIO.demand = turretAngleToEncUnits(desiredTurretAngle) + offset;
        //mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
        mTurretMotor.set(ControlMode.Position, periodicIO.demand);
    }

    public void outputTelemetry() {
        if (TurretConstants.kOutputTelemetry) {
            SmartDashboard.putNumber("Debug/Turret/Encoder", periodicIO.position);
            SmartDashboard.putNumber("Debug/Turret/Demand", periodicIO.demand);
            SmartDashboard.putNumber("Debug/Turret/Error",
                    Conversions.talonToDegrees(mTurretMotor.getClosedLoopError(0), TurretConstants.TURRET_GEAR_RATIO));
            if (mTurretMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber("Debug/Turret/Setpoint", mTurretMotor.getClosedLoopTarget(0));
        }
    }

    public enum TurretControlState {
        HOME, ON, STOP
    }

    public static class PeriodicIO {
        // Inputs
        public double position;
        public int velocity;
        public double voltage;
        public double current;

        // Outputs
        public double demand;
    }

    @Override
    public void periodic() {
        writePeriodicOutputs();
        readPeriodicInputs();
        outputTelemetry();
    }

    //here angel is radian
    public double getMeasurement() {
        return getAngleRad();
    }
    
    public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision) {
        Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
        double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());
        angle = Math.PI + angle - robotPose.getRotation().getRadians();
    
        angle = MathUtils.toUnitCircAngle(angle);
    
        if (m_trackTarget) {
            limelight.setLightMode(3);
        } else {
            limelight.setLightMode(1);
        }
    
        SmartDashboard.putNumber("Turret Set Angle", angle);
    
        if (aimAtVision && limelight.isTargetVisible()) {
          angle = getMeasurement() - Math.toRadians(limelight.Get_tx());
        }
    
        setTurretAngle(Math.toDegrees(angle));
    }
    
    /**
       * Function to set the track target boolean to either true or false
       * 
       * @param track is true when Limelight tracking is desired
    */
    public void trackTarget(boolean track) {
        m_trackTarget = track;
    }
    
    public boolean visionAligned() {
        if (limelight.isTargetVisible() && Math.abs(limelight.Get_tx()) < LimelightConstants.kTrackTolerance) {
          return true;
        } else {
          return false;
        }
      }
    
    public boolean atDesiredAngle() {
        return Math.abs(desiredTurretAngle - Math.toDegrees(getMeasurement())) <= TurretConstants.kTolerance;
    }
    
}
