package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import edu.wpi.first.wpilibj.RobotBase;

public final class Hopper extends SubsystemBase  {

    private static Hopper instance ;
    WPI_TalonFX hopperMotor;
    private final DigitalInput bottomIRSensor;   /* 位于球道底部，用于判断intake是否已经收入一个新球进来 */
    private final DigitalInput topIRSensor; /* 位于球道顶部，用于判断是否有球在顶部*/
    private int testMode1 = 0;
    private int testMode2 = 0;

    public static Hopper getInstance() {
        if (instance == null){
            instance = new Hopper();
        }
        return instance;
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }
    HopperState hopperState = HopperState.OFF;


    private Hopper() {
        //super(Constants.HOPPER_PERIOD, 5);
        hopperMotor = new WPI_TalonFX(HopperConstants.HopperPort); //TODO
        hopperMotor.configVoltageCompSaturation(12);
        hopperMotor.enableVoltageCompensation(true);
        topIRSensor = new DigitalInput(HopperConstants.HOPPER_TOP_BALL_IR_SENSOR); //TODO
        bottomIRSensor = new DigitalInput(HopperConstants.HOPPER_LOW_BALL_IR_SENSOR); //TODO
        hopperState = HopperState.OFF;
    }

    @Override
    public void periodic() {
        // Hopper Motor Control
        switch (hopperState) {
            case ON:
                setHopperSpeed(HopperConstants.HOPPER_SPEED);
                break;
            case OFF:
                setHopperSpeed(0);
                break;
            case REVERSE:
                setHopperSpeed(-HopperConstants.HOPPER_SPEED);
                break;
            case SLOW:
                setHopperSpeed(HopperConstants.HOPPER_SLOW_SPEED);
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

    public void setHopperState(HopperState state) {
        hopperState = state;
    }

    public HopperState getHopperState() {
        return hopperState;
    }

    public void outputTelemetry(){
        SmartDashboard.putNumber("Debug/Hopper/Motor Output", hopperMotor.getMotorOutputPercent());
        SmartDashboard.putString("Debug/Hopper/State", hopperState.toString());
    }

    public boolean isHasTopBall() {
        if (RobotBase.isSimulation()){
            return (testMode1 == 0) ? false:true;
        }
        return !topIRSensor.get();
    }

    public boolean isHasBottomBall() {
        if (RobotBase.isSimulation()){
            return (testMode2 == 0) ? false:true;
        }
        return !bottomIRSensor.get();
    }

    public void DotestMode(){
        testMode1 = (testMode1 == 0) ? 1:0;
    }

}
