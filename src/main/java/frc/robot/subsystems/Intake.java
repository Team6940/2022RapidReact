package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.team3476.Timer;
import frc.robot.subsystems.Hopper.*;
import edu.wpi.first.wpilibj.RobotBase;


public class Intake extends SubsystemBase {

    private static Intake instance ;
    private Solenoid intakeSolenoid;
    private WPI_TalonFX intakeMotor;

    private double allowIntakeRunTime = Double.MAX_VALUE;

    public static Intake getInstance() {
        if (instance == null){
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        intakeMotor = new WPI_TalonFX(Constants.IntakerPort+10); //TODO
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakerSolenoidPort); //TODO
        intakeMotor.configVoltageCompSaturation(12);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 97); // Default is 10ms
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 101); // Default is 10ms
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 103); // Default is 50ms
        intakeMotor.setControlFramePeriod(ControlFrame.Control_3_General, 23);
        intakeMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 29);
        intakeMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 547);
        intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 70, 0), 1000);
        intakeMotor.configOpenloopRamp(0.2, 1000);
    }

    public void selfTest() {
        setIntakeSolState(IntakeSolState.OPEN);
        //OrangeUtility.sleep(1000);
        setIntakeState(IntakeState.INTAKE);
        //OrangeUtility.sleep(3000);
        setIntakeState(IntakeState.OFF);
        setIntakeSolState(IntakeSolState.CLOSE);
    }

    public void outputTelemetry(){
        //SmartDashboard.putNumber("Debug/Intake/Current", intakeMotor.getStatorCurrent());
        SmartDashboard.putBoolean("Debug/Intake/IntakeSolState: ", intakeSolenoid.get());
        SmartDashboard.putString("Debug/Intake/IntakeSolState", wantedIntakeSolState.toString());
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", intakeMotor.getMotorOutputPercent());
        SmartDashboard.putString("Debug/Intake/WantedIntake State", getWantedIntakeState().name());
    }


    public void close() {
        intakeSolenoid.close();
        intakeMotor.close();
        instance = new Intake();
    }

    public IntakeSolState getIntakeSolState() {
        if (RobotBase.isSimulation()){
            return wantedIntakeSolState;
        }else{
            return intakeSolenoid.get() ? IntakeSolState.OPEN : IntakeSolState.CLOSE;
        }
  
    }

    // Intake States

    public enum IntakeSolState {
        OPEN, CLOSE
    }

    private IntakeSolState wantedIntakeSolState = IntakeSolState.CLOSE;

    // this a extern func for other command call.
    public synchronized void setIntakeSolState(IntakeSolState intakeSolState) {
        wantedIntakeSolState = intakeSolState;
        switch (intakeSolState) {
            case OPEN:
                intakeSolenoid.set(true);
                break;
            case CLOSE:
                intakeSolenoid.set(false);
        }
    }

    public enum IntakeState {
        INTAKE, EJECT, SLOW_EJECT, OFF
    }

    private IntakeState wantedIntakeState = IntakeState.OFF;
    
    // this a extern func for other command call.
    public synchronized void setWantedIntakeState(IntakeState intakeState) {
        wantedIntakeState = intakeState;
    }
    public synchronized IntakeState getWantedIntakeState() {
        return wantedIntakeState ;
    }

    private void setIntakeMotor(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    private void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case INTAKE:
                setIntakeMotor(Constants.INTAKE_SPEED);
                break;
            case OFF:
                setIntakeMotor(0);
                break;
        }
    }

    @Override
    public void periodic() {
        setIntakeState(wantedIntakeState);
        outputTelemetry();
    }

    private int cnt = 0;
    public void autoturnintaker()
    {
        if(cnt % 2 == 0){
             setIntakeSolState(IntakeSolState.OPEN);
             setWantedIntakeState(IntakeState.INTAKE);
             Hopper.getInstance().setHopperState(HopperState.ON);
        }
        else{
            setIntakeSolState(IntakeSolState.CLOSE);
            setWantedIntakeState(IntakeState.OFF);
            Hopper.getInstance().setHopperState(HopperState.OFF);
        }
        cnt++;

    }
    
}