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
import frc.robot.Constants.IntakeConstants;
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
        intakeMotor = new WPI_TalonFX(IntakeConstants.IntakerPort); //TODO 设定intake电机
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.IntakerSolenoidPort); //TODO 设定气动杆
        //intakeMotor.configVoltageCompSaturation(10);
        //intakeMotor.enableVoltageCompensation(true);
        //intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 97); // Default is 10ms
        //intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 101); // Default is 10ms
        //intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 103); // Default is 50ms
        //intakeMotor.setControlFramePeriod(ControlFrame.Control_3_General, 23);
        //intakeMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 29);
        //intakeMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 547);
        //intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 40, 70, 0), 1000);
        //intakeMotor.configOpenloopRamp(0.2, 1000);
        //configStatusFramePeriods();
    }

    public void configStatusFramePeriods() {
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 19);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 253);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 59);
    }

    public void selfTest() {//自我测试，把intake打开旋转再关上停转
        setIntakeSolState(IntakeSolState.OPEN);
        //OrangeUtility.sleep(1000);
        setIntakeState(IntakeState.INTAKE);
        //OrangeUtility.sleep(3000);
        setIntakeState(IntakeState.OFF);
        setIntakeSolState(IntakeSolState.CLOSE);
    }

    public void outputTelemetry(){
        //SmartDashboard.putNumber("Debug/Intake/Current", intakeMotor.getStatorCurrent());
        SmartDashboard.putBoolean("Debug/Intake/IntakeSolState: ", intakeSolenoid.get());//在调试版上显示气动杆状态
        SmartDashboard.putString("Debug/Intake/IntakeSolState", wantedIntakeSolState.toString());//在调试版上显示气动杆的目标状态
        SmartDashboard.putNumber("Debug/Intake/CMotorOutput: ", intakeMotor.getMotorOutputPercent());//在调试板上显示电机输出百分比
        SmartDashboard.putString("Debug/Intake/WantedIntake State", getWantedIntakeState().name());//在调试版上输出intake的目标状态
    }


    public void close() {//关闭intake
        intakeSolenoid.close();//关闭起动杆
        intakeMotor.close();//关闭intake电机
        instance = new Intake();//???
    }

    public IntakeSolState getIntakeSolState() {//???
        if (RobotBase.isSimulation()){//???
            return wantedIntakeSolState;
        }else{
            return intakeSolenoid.get() ? IntakeSolState.OPEN : IntakeSolState.CLOSE;
        }
  
    }

    // Intake States
    public enum IntakeSolState {//气动杆状态，打开和关闭
        OPEN, CLOSE
    }

    private IntakeSolState wantedIntakeSolState = IntakeSolState.CLOSE;//默认设定intake起动杆为关闭

    // this a extern func for other command call.
    public synchronized void setIntakeSolState(IntakeSolState intakeSolState) {//将气动杆状态输出到气动杆
        wantedIntakeSolState = intakeSolState;
        switch (intakeSolState) {
            case OPEN://打开气动杆
                intakeSolenoid.set(false);
                break;
            case CLOSE://关闭气动杆
                intakeSolenoid.set(true);
        }
    }

    public enum IntakeState {//intake状态，分别对应：吸球、吐出、缓慢吐出、关闭
        INTAKE, EJECT, SLOW_EJECT, OFF
    }

    private IntakeState wantedIntakeState = IntakeState.OFF;//默认intake状态关闭
    
    // this a extern func for other command call.
    public synchronized void setWantedIntakeState(IntakeState intakeState) {//设定intake状态
        wantedIntakeState = intakeState;
    }
    public synchronized IntakeState getWantedIntakeState() {//获取intake状态
        return wantedIntakeState ;
    }

    private void setIntakeMotor(double speed) {//设定intake电机转速
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    private void setIntakeState(IntakeState intakeState) {//将intake状态输出到电机上
        switch (intakeState) {
            case INTAKE:
                setIntakeMotor(IntakeConstants.INTAKE_SPEED);
                break;
            case OFF:
                setIntakeMotor(0);
                break;
        }
    }

    @Override
    public void periodic() {
        setIntakeState(wantedIntakeState);//持续地将Intake状态输出到电机上
        outputTelemetry();
    }

    public void runIntaker() {
        setIntakeSolState(IntakeSolState.OPEN);
        setWantedIntakeState(IntakeState.INTAKE);
        //Hopper.getInstance().setHopperState(HopperState.ON);
    }

    public void stopIntaker() {
        //setIntakeSolState(IntakeSolState.CLOSE);
        setWantedIntakeState(IntakeState.OFF);
        //Hopper.getInstance().setHopperState(HopperState.OFF);
    }
    
    public void stopSolenoid() {
        setIntakeSolState(IntakeSolState.CLOSE);
    }

    private int cnt = 0;
    public void autoturnintaker()//用cnt周期性地控制intake状态
    {
        if(cnt % 2 == 0){
             setIntakeSolState(IntakeSolState.CLOSE);
             setWantedIntakeState(IntakeState.INTAKE);
             Hopper.getInstance().setHopperState(HopperState.ON);
        }
        else{
            setIntakeSolState(IntakeSolState.OPEN);
            setWantedIntakeState(IntakeState.OFF);
            Hopper.getInstance().setHopperState(HopperState.OFF);
        }
        cnt++;
    }

    public boolean isIntakerOn(){
        return getWantedIntakeState() == IntakeState.INTAKE ?true:false;
    }
}