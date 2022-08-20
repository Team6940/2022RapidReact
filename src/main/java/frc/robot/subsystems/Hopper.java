package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.subsystems.Intake.IntakeSolState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.team3476.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class Hopper extends SubsystemBase  {

    private static Hopper instance ;
    WPI_TalonFX hopperMotor;
    private final ColorSensor colorSensor;  /* 位于球道顶部，用于识别当前将要给shooter发射的球颜色 */
    private boolean isBeamBreakEnabled = true;
    private final DigitalInput beamBreak;   /* 位于球道底部，用于判断intake是否已经收入一个新球进来 */
    private final DigitalInput topIRSensor; /* 位于球道顶部，用于判断是否有球在顶部*/
    private double lastBeamBreakOpenTime = 0;

    public static Hopper getInstance() {
        if (instance == null){
            instance = new Hopper();
        }
        return instance;
    }

    public void resetBeamBreakOpenTime() {
        lastBeamBreakOpenTime = Timer.getFPGATimestamp();
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }
    HopperState hopperState = HopperState.OFF;

    public enum BallColor {
        RED,
        BLUE,
        NO_BALL
    }

    BallColor opposingAllianceColor = BallColor.BLUE;
    BallColor friendlyAllianceColor = BallColor.RED;

    private Hopper() {
        //super(Constants.HOPPER_PERIOD, 5);
        hopperMotor = new WPI_TalonFX(Constants.BallLoaderPort+10); //TODO
        hopperMotor.configVoltageCompSaturation(12);
        hopperMotor.enableVoltageCompensation(true);
        topIRSensor = new DigitalInput(Constants.HOPPER_TOP_BALL_IR_SENSOR); //TODO
        beamBreak =  new DigitalInput(Constants.HOPPER_LOW_BALL_IR_SENSOR);  //TODO
        colorSensor = ColorSensor.getInstance();  //TODO
    }
/**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if( DriverStation.getAlliance() == Alliance.Blue){
            opposingAllianceColor = BallColor.RED;
            friendlyAllianceColor = BallColor.BLUE;
        } else {
            opposingAllianceColor = BallColor.BLUE;
            friendlyAllianceColor = BallColor.RED;
        }
    }


    public BallColor getBallColor() {
        BallColor currentBallColor;
        if( colorSensor.getCurrentBall() == ColorSensor.BallColor.BLUE_BALL){
            currentBallColor = BallColor.BLUE;
        }else if (colorSensor.getCurrentBall() == ColorSensor.BallColor.RED_BALL){
            currentBallColor = BallColor.RED;
        }else {
            currentBallColor = BallColor.NO_BALL;
        }
        return currentBallColor;
    }


    public double getLastBeamBreakOpenTime() {
        return lastBeamBreakOpenTime;
    }

    @Override
    public void periodic() {
        updateAllianceColor();

        if (!isBeamBroken()) {
            resetBeamBreakOpenTime();
        }

        // Hopper Motor Control
        switch (hopperState) {
            case ON:
                setHopperSpeed(Constants.HOPPER_SPEED);
                break;
            case OFF:
                setHopperSpeed(0);
                break;
            case REVERSE:
                setHopperSpeed(-Constants.HOPPER_SPEED);
                break;
            case SLOW:
                setHopperSpeed(Constants.HOPPER_SLOW_SPEED);
                break;
        } //TODO

        outputTelemetry();
    }

    private void setHopperSpeed(double speed) {
        synchronized (hopperMotor) {
            hopperMotor.set(speed);
        }
    }

    public boolean isHopperMotorRunning() {
        synchronized (hopperMotor) {
            return hopperMotor.get() != 0;
        }
    }

    /**
     * @return True if the beam break is broken. (ie a ball is in the way)
     */
    public boolean isBeamBroken() {
        if (!isBeamBreakEnabled) return false;
        return !beamBreak.get();
    }


    public boolean isBeamBreakEnabled() {
        return isBeamBreakEnabled;
    }

    public void setBeamBreakEnabled(boolean isBeamBreakEnabled) {
        this.isBeamBreakEnabled = isBeamBreakEnabled;
    }

    public void setHopperState(HopperState state) {
        hopperState = state;
    }

    public HopperState getHopperState() {
        return hopperState;
    }

    public void selfTest() {
        setHopperState(HopperState.ON);
        // OrangeUtility.sleep(5000);
        setHopperState(HopperState.OFF);
        //OrangeUtility.sleep(5000);

    }

    public void outputTelemetry(){
        SmartDashboard.putNumber("Debug/Hopper/Motor Output", hopperMotor.getMotorOutputPercent());
        SmartDashboard.putString("Debug/Hopper/Current Ball Color", getBallColor().toString());
        SmartDashboard.putString("Debug/Hopper/State", hopperState.toString());
        SmartDashboard.putBoolean("Debug/Hopper/Is Beam Broken", isBeamBroken());
        SmartDashboard.putNumber("Debug/Hopper/Last Beam Break Open Time", getLastBeamBreakOpenTime());
    }

    public void close(){
        setHopperState(HopperState.OFF);
        instance = new Hopper();
    }
}
