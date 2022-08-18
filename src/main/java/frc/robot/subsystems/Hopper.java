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
    private double lastDetectionTime;
    private boolean disableEject = false;
    private final ColorSensor2 colorSensor;  /* 位于球道顶部，用于识别当前将要给shooter发射的球颜色 */
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
/**
     * Outtake can either be OFF or INTAKE, or AUTO_EJECT
     */
    public enum OuttakeState {
        OFF,
        INTAKE,
        AUTO_EJECT,
        MANUAL_EJECT
    }

    private OuttakeState outtakeState = OuttakeState.OFF;

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
        topIRSensor = new DigitalInput(Constants.TOP_BALL_IR_SENSOR); //TODO
        beamBreak =  new DigitalInput(Constants.LOW_BALL_IR_SENSOR+5);  //TODO
        colorSensor = new ColorSensor2();
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

    /**
     * Toggles disableEject
     */
    public void toggleEjectOverride() {
        disableEject = !disableEject;
    }

    /**
     * Toggles disableEject
     */
    public void setEjectOverride(boolean ejectOverride) {
        disableEject = ejectOverride;
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        Intake intake = Intake.getInstance();

        if (hopperState == HopperState.REVERSE) {
            outtakeState = intake.getIntakeSolState() == IntakeSolState.OPEN ? OuttakeState.MANUAL_EJECT : OuttakeState.OFF;
            return;
        }
        if (hopperState == HopperState.OFF) {
            outtakeState = OuttakeState.OFF;
            return;
        }

        if (getBallColor() == opposingAllianceColor
                && intake.getIntakeSolState() == IntakeSolState.OPEN
                && !disableEject) { // If disable eject is on, it will not outtake
            lastDetectionTime = Timer.getFPGATimestamp();
            outtakeState = OuttakeState.AUTO_EJECT;
        } else if (Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD
                && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            // Ensure that we keep outtaking for a minimum amount of time
            outtakeState = OuttakeState.AUTO_EJECT;
        } else {
            outtakeState = OuttakeState.INTAKE;
        }
    }

    /**
     * Sets the percent outtake between 1 and -1
     */
    private void setOuttakePercentOutput(double percentOutput) {
        //outtakeWheels.set(percentOutput);
    }

    public OuttakeState getOuttakeState() {
        return outtakeState;
    }

    public BallColor getBallColor() {
        BallColor currentBallColor;
        if( colorSensor.getCurrentBall() == ColorSensor2.BallColor.BLUE_BALL){
            currentBallColor = BallColor.BLUE;
        }else if (colorSensor.getCurrentBall() == ColorSensor2.BallColor.RED_BALL){
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
        updateOuttakeState();

        if (!isBeamBroken()) {
            resetBeamBreakOpenTime();
        }

        // Outtake motor control

        switch (outtakeState) {
            case OFF:
                setOuttakePercentOutput(0);
                break;
            case AUTO_EJECT:
                setOuttakePercentOutput(Constants.OUTTAKE_AUTO_EJECTION_SPEED);
                break;
            case INTAKE:
                setOuttakePercentOutput(Constants.OUTTAKE_SPEED);
                break;
            case MANUAL_EJECT:
                setOuttakePercentOutput(Constants.OUTTAKE_MANUAL_EJECTION_SPEED);
                break;
        }

        // Hopper Motor Control
        switch (hopperState) {
            case ON:
                if (outtakeState == OuttakeState.AUTO_EJECT
                    //&& Shooter.getInstance().getFeederWheelState() != FeederWheelState.FORWARD) 
                    && (Shooter.getInstance().getBlockerState() != Shooter.BlockerControlState.BALLLOCKER_ON)){
                    setHopperSpeed(Constants.HOPPER_OUTTAKING_SPEED);
                } else {
                    setHopperSpeed(Constants.HOPPER_SPEED);
                }
                break;
            case OFF:
                if (isBeamBroken() &&
                        !(Shooter.getInstance().getBlockerState() == Shooter.BlockerControlState.BALLLOCKER_ON &&
                                Shooter.getInstance().getShooterState() == Shooter.ShooterControlState.SHOOT)) {
                    //If a ball is blocking the beam break sensor we want to run the hopper to move the ball up to unblock it.
                    setHopperSpeed(Constants.HOPPER_SPEED);
                } else {
                    setHopperSpeed(0);
                }
                break;
            case REVERSE:
                setHopperSpeed(-Constants.HOPPER_SPEED);
                break;
            case SLOW:
                setHopperSpeed(Constants.HOPPER_SLOW_SPEED);
                break;
        }

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

        setOuttakePercentOutput(-Constants.INTAKE_EJECTION_SPEED);
        System.out.println("Ejecting");
        //OrangeUtility.sleep(Constants.TEST_TIME_MS);

        setOuttakePercentOutput(Constants.INTAKE_EJECTION_SPEED);
        System.out.println("Intaking");
        //OrangeUtility.sleep(Constants.TEST_TIME_MS);

        System.out.println("Test Finished");
        setOuttakePercentOutput(0);
    }

    public void outputTelemetry(){
        SmartDashboard.putNumber("Debug/Hopper/Motor Output", hopperMotor.getMotorOutputPercent());
        SmartDashboard.putString("Debug/Hopper/Outtake State", outtakeState.toString());
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
