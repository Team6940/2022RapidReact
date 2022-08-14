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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.team1678.math.Conversions;
import frc.robot.lib.team1690.Vector;
import frc.robot.lib.team1690.Vector3D;
import frc.robot.lib.team2910.math.Vector2;
import frc.robot.lib.team2910.util.InterpolatingDouble;
import frc.robot.lib.team2910.util.InterpolatingTreeMap;
import frc.robot.lib.team503.util.Util;
import edu.wpi.first.wpilibj.RobotBase;


public class Shooter2 extends SubsystemBase {
  
    static InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();
    private static Shooter2 instance = null;
    private int num = 1;
    private int shootMode = 1;//1 means table shoot ,0 means algorithm shoot
    private boolean ShotWhileMove = true;
    private static WPI_TalonFX mShooter;
    private final ShooterPeriodicIO ShooterPeriodicIO = new ShooterPeriodicIO();
    ShooterControlState shootState = ShooterControlState.STOP;
    private double MoveOffset = 0;
    private boolean readyToShoot = false;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    private boolean velocityStabilized = true;
    double targetVelocity ;
    double targetHoodAngle ;
    double targetTurretAngle;
    private double ShooterDesiredSpeed = 0;

    /**
     * We use FeedForward and a little P gains for Shooter
     */
    SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);
    private ShuffleboardTab shooterSpeedTab = Shuffleboard.getTab("Shooter Speed");
    private NetworkTableEntry shooterSpeed =
        shooterSpeedTab.add("Shooter Speed", 1).getEntry();
    private ShuffleboardTab HoodAngleTab = Shuffleboard.getTab("Hood Angle");
    private NetworkTableEntry hoodAngle =
        HoodAngleTab.add("Hood Angle", 1).getEntry();

    static { //TODO first is meters for distance,second is RPM
        SHOOTER_TUNING.put(new InterpolatingDouble(0.0000), new Vector2(70,3000));//TODO
        SHOOTER_TUNING.put(new InterpolatingDouble(1.9304), new Vector2(55,3500));
        SHOOTER_TUNING.put(new InterpolatingDouble(2.5400), new Vector2(50,4000));
        SHOOTER_TUNING.put(new InterpolatingDouble(3.3020), new Vector2(45,4500));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0640), new Vector2(40,5000));
        SHOOTER_TUNING.put(new InterpolatingDouble(4.8260), new Vector2(35,5500));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0960), new Vector2(30,6000));
        SHOOTER_TUNING.put(new InterpolatingDouble(7.6200), new Vector2(25,6500));
    }

    // for hood
    private WPI_TalonSRX mHoodmotor;
    private static final double kEncMin = 0.0;//TODO
    private static final double kEncMax = 627;
    private int offset = 0;
    HoodPeriodicIO HoodPeriodicIO = new HoodPeriodicIO();
    private double HoodDesiredAngle;
  
    // for blocker
    private  Solenoid blockerSolenoid;

    public Shooter2() {
        configShooter();
        configHood();
        configBlocker();
    }

    public static Shooter2 getInstance() {
        if (instance == null){
            instance = new Shooter2();
        }
        return instance;
    }

    private void configShooter() {
        TalonFXConfiguration lMasterConfig = new TalonFXConfiguration();

        lMasterConfig.slot0.kP = 1.5;//TODO
        lMasterConfig.slot0.kI = 0;
        lMasterConfig.slot0.kD = 0;
        lMasterConfig.slot0.kF = 0;
        lMasterConfig.peakOutputForward = 1.0;
        lMasterConfig.peakOutputReverse = 0.0;
        mShooter = new WPI_TalonFX(Constants.SHOOT_L_MASTER_ID);
        mShooter.configAllSettings(lMasterConfig);
        mShooter.setNeutralMode(NeutralMode.Coast);
        mShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mShooter.configVoltageCompSaturation(12);
        mShooter.enableVoltageCompensation(true);
    }

    private void configHood(){
        mHoodmotor = new WPI_TalonSRX(Constants.HoodMotorPort);

        mHoodmotor.setInverted(false);
        mHoodmotor.setSensorPhase(false);//TODO
        mHoodmotor.setNeutralMode(NeutralMode.Brake);
    
        mHoodmotor.selectProfileSlot(0, 0);//TODO
        mHoodmotor.config_kP(0, 3.66, 10);
        mHoodmotor.config_kI(0, 0.01, 10);
        mHoodmotor.config_kD(0, 10.0, 10);
        mHoodmotor.config_kF(0, 0.00, 10);
        mHoodmotor.config_IntegralZone(0, 15, 10);
        mHoodmotor.configMotionCruiseVelocity(600, 10);
        mHoodmotor.configMotionAcceleration(1200, 10);
        // mHoodmotor.configMotionSCurveStrength(6);

        mHoodmotor.configForwardSoftLimitThreshold(Conversions.degreesToTalon(kEncMin, Constants.HOOD_GEAR_RATIO), 10); //TODO
        mHoodmotor.configReverseSoftLimitThreshold(Conversions.degreesToTalon(kEncMax, Constants.HOOD_GEAR_RATIO), 10); //TODO
        mHoodmotor.configForwardSoftLimitEnable(true, 10);
        mHoodmotor.configReverseSoftLimitEnable(true, 10);
    
        mHoodmotor.configVoltageCompSaturation(12);
        mHoodmotor.enableVoltageCompensation(true);
    
        mHoodmotor.configPeakOutputForward(0.50, 10);//TODO
        mHoodmotor.configPeakOutputReverse(-0.50, 10);//TODO
    
        mHoodmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    }

    private void configBlocker(){
        blockerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakerSolenoidPort+6); //TODO
    }

    public ShooterControlState getState() {
      return shootState;
    }

    public void ShooterOutputTelemetry() {
        SmartDashboard.putNumber("Debug/Shooter/Mode", shootMode);
        SmartDashboard.putString("Debug/Shooter/State", shootState.name());   
        SmartDashboard.putNumber("Debug/Shooter/Velocity(RPM)", ShooterPeriodicIO.flywheel_velocity);
        SmartDashboard.putNumber("Debug/Shooter/targetHoodAngle", targetHoodAngle);
        SmartDashboard.putNumber("Debug/Shooter/targetTurretAngle", targetTurretAngle);
    }

    public void ShooterReadPeriodicInputs() {
        ShooterPeriodicIO.timestamp = Timer.getFPGATimestamp();
            
        ShooterPeriodicIO.flywheel_velocity = getShooterSpeedRpm();
        ShooterPeriodicIO.flywheel_voltage = mShooter.getMotorOutputVoltage();
        ShooterPeriodicIO.flywheel_current = mShooter.getSupplyCurrent();
        ShooterPeriodicIO.flywheel_temperature = mShooter.getTemperature();

    }

    public void ShooterWritePeriodicOutputs() {
        if(shootState == ShooterControlState.STOP) {
            mShooter.set(ControlMode.PercentOutput, 0);
            setHoodStop();
        }
        if(shootState == ShooterControlState.INIT) {
            mShooter.set(ControlMode.PercentOutput, 0);
        }
        if(shootState == ShooterControlState.PREPARE_SHOOT){
            double cal_shooterFeedForward = shooterFeedForward.calculate(Conversions.RPMToMPS(ShooterDesiredSpeed, Constants.kFlyWheelCircumference));
            ShooterPeriodicIO.flywheel_demand = Conversions.RPMToFalcon(ShooterDesiredSpeed,Constants.kFlyWheelEncoderReductionRatio);
            mShooter.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
        }
        if(shootState == ShooterControlState.SHOOT){
            double cal_shooterFeedForward = shooterFeedForward.calculate(Conversions.RPMToMPS(ShooterDesiredSpeed, Constants.kFlyWheelCircumference));
            ShooterPeriodicIO.flywheel_demand = Conversions.RPMToFalcon(ShooterDesiredSpeed,Constants.kFlyWheelEncoderReductionRatio);
            mShooter.set(ControlMode.Velocity, ShooterPeriodicIO.flywheel_demand, DemandType.ArbitraryFeedForward, cal_shooterFeedForward);
            if(isShooterCanShoot()){  
                setFiring(true);
            }
        }
    }

    public int getShooterMode(){
        return shootMode;
    }

    public void setFixedShootParams(){
        double  speed = Conversions.MPSToRPM( getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE),
                                     Constants.kFlyWheelCircumference);
        setSpeed(speed);
    }

    public void SetMovingShootParams(){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(LimelightSubsystem.getInstance().getRobotToTargetDistance_Opt().orElse(0.0)));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                Conversions.RPMToMPS(angleAndSpeed.y, Constants.kFlyWheelCircumference), // Shot Speed
                angleAndSpeed.x,                                                         // Hood Angle
                SwerveDriveTrain.getInstance().getFieldRelativeTurretAngleDeg(),         // Turret target angle (The same coordinate system with swerve)
                SwerveDriveTrain.getInstance().getFieldRelativeXVelocity(),              // Swerve speed in X axis (field-oriented)
                SwerveDriveTrain.getInstance().getFieldRelativeYVelocity());             // Swerve speed in Y axis (field-oriented)
        targetVelocity = Conversions.MPSToRPM(mShotParams[2], Constants.kFlyWheelCircumference);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - SwerveDriveTrain.getInstance().GetHeading_Deg());
        MoveOffset = targetTurretAngle - Turret.getInstance().getAngleDeg();
        setSpeed(targetVelocity);
        setHoodAngle(targetHoodAngle);
    }

    public boolean isShooterCanShoot(){
        boolean result = false;
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(LimelightSubsystem.getInstance().getRobotToTargetDistance_Opt().orElse(0.0)));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                Conversions.RPMToMPS(angleAndSpeed.y, Constants.kFlyWheelCircumference), // Shot Speed   
                angleAndSpeed.x,                                                         // Hood Angle
                SwerveDriveTrain.getInstance().getFieldRelativeReadyTurretAngleDeg(),    // Turret Angle (The same coordinate system with swerve)
                SwerveDriveTrain.getInstance().getFieldRelativeXVelocity(),              // Swerve speed in X axis (field-oriented)
                SwerveDriveTrain.getInstance().getFieldRelativeYVelocity());             // Swerve speed in Y axis (field-oriented)
        targetVelocity = Conversions.MPSToRPM(mShotParams[2],Constants.kFlyWheelCircumference);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - SwerveDriveTrain.getInstance().GetHeading_Deg());
        result = (Math.abs(targetVelocity - getShooterSpeedRpm()) < 50) 
                 &&( Math.abs(targetTurretAngle - Turret.getInstance().getAngleDeg()) < 1.0)
                 &&( Math.abs(targetHoodAngle - getHoodAngle()) < 1.0)
                ; 
        return result;
    }

    public Vector3D calcBallVelocity() {
        //float vel = (float)SwerveDriveTrain.getInstance().getRadialVelocity();
        //float dist = (float)LimelightSubsystem.getInstance().getRobotToTargetDistance();
        float vel = (float) -3.8;
        float dist = (float) 5.0;

        if (vel < 0) {
            vel *= 1.09f;
        } else {
            vel *= 1.06f;
        }

        final float distSquared = dist * dist;
        final float disCubed = distSquared * dist;

        final float velSquared = vel * vel;
        final float velCubed = velSquared * vel;

        final float distVel = dist * vel;
        final float distSquaredVel = distVel * dist;
        final float velSquaredDist = distVel * vel;

        float pitch = 0;
        pitch += Constants.angleCoefficients[0] * disCubed;
        pitch += Constants.angleCoefficients[1] * distSquaredVel;
        pitch += Constants.angleCoefficients[2] * velSquaredDist;
        pitch += Constants.angleCoefficients[3] * velCubed;
        pitch += Constants.angleCoefficients[4] * distSquared;
        pitch += Constants.angleCoefficients[5] * dist;
        pitch += Constants.angleCoefficients[6] * velSquared;
        pitch += Constants.angleCoefficients[7] * vel;
        pitch += Constants.angleCoefficients[8] * distVel;
        pitch += Constants.angleCoefficients[9];

        float speed = 0;
        speed += Constants.speedCoefficients[0] * disCubed;
        speed += Constants.speedCoefficients[1] * distSquaredVel;
        speed += Constants.speedCoefficients[2] * velSquaredDist;
        speed += Constants.speedCoefficients[3] * velCubed;
        speed += Constants.speedCoefficients[4] * distSquared;
        speed += Constants.speedCoefficients[5] * dist;
        speed += Constants.speedCoefficients[6] * velSquared;
        speed += Constants.speedCoefficients[7] * vel;
        speed += Constants.speedCoefficients[8] * distVel;
        speed += Constants.speedCoefficients[9];

        final float tangentialRobotSpeed = (float)SwerveDriveTrain.getInstance().getTangentialVelocity();
        final Vector tangentialRobotVel = Vector.fromAngleAndRadius(
                (float)SwerveDriveTrain.getInstance().getFieldRelativeTurretAngleRad() + (float)Math.toRadians(90),
                tangentialRobotSpeed);
        final Vector3D tangentialRobotVel3D = new Vector3D(tangentialRobotVel.x, tangentialRobotVel.y, 0);

        final float angleToTargetWithoutRobotVel = (float)SwerveDriveTrain.getInstance().getFieldRelativeTurretAngleRad();

        final Vector3D ballVelocityWithoutYawLimit = Vector3D
                .fromSizeYawPitch(speed, angleToTargetWithoutRobotVel, pitch)
                .scale(1 + tangentialRobotVel.norm() * 0.005f).subtract(tangentialRobotVel3D);

        final Vector3D ballVelocity = Vector3D.fromSizeYawPitch(ballVelocityWithoutYawLimit.norm(),
                ballVelocityWithoutYawLimit.getYaw(),
                ballVelocityWithoutYawLimit.getPitch());

        return ballVelocity;
    }
    
    public void shootOnMoveOrbit() {
        Vector3D ballVelocity = calcBallVelocity();
        targetVelocity = Conversions.MPSToRPM(ballVelocity.norm(),Constants.kFlyWheelCircumference);
        targetHoodAngle = Math.toDegrees(ballVelocity.getPitch());
        targetTurretAngle = Util
                .boundAngleNeg180to180Degrees(ballVelocity.getYaw() - SwerveDriveTrain.getInstance().GetHeading_Deg());
        MoveOffset = targetTurretAngle - Turret.getInstance().getAngleDeg();
        setSpeed(targetVelocity);
        setHoodAngle(targetHoodAngle);
    }

    public boolean shootIsReady(){
        return isShooterCanShoot();
    }

    public synchronized double getShooterSpeedRpm() {
        if (RobotBase.isSimulation()) {
            return Conversions.falconToRPM(ShooterPeriodicIO.flywheel_demand, Constants.kFlyWheelEncoderReductionRatio);
        }else{
            return Conversions.falconToRPM(mShooter.getSelectedSensorVelocity(), Constants.kFlyWheelEncoderReductionRatio);
        }

    }

    public double getShooterLaunchVelocity(double shooterAngle) {
        double speed = 0;
        double d = Constants.CARGO_DIAMETER;
        double D = Constants.UPPER_HUB_DIAMETER;
        double g = 9.81;
        double H = Constants.LL_UPPER_HUB_HEIGHT;
        double h = Constants.SHOOTER_MOUNT_HEIGHT;
        double L = LimelightSubsystem.getInstance().getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2; //getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2 ; 
        double alpha = Math.toRadians(shooterAngle); // Set to proper value
        /* v is mini speed  */
        double vMin = Math.sqrt(g * (H-h+Math.sqrt(Math.pow(L,2)+Math.pow(H-h,2))));
        double v = L / Math.cos(alpha) * Math.sqrt( g / (2 * ( L *Math.tan(alpha) - H + h )));
        double beta = Math.toDegrees(Math.atan(Math.tan(alpha) - 2*(H-h) / L));
        double betaMinLimit = Math.toDegrees(Math.asin(d/D));
        if( (v >= vMin) && (beta >= betaMinLimit )){
        speed = v;
        }
        SmartDashboard.putNumber("Debug/Shooter/calcShootSpeed", speed);
        return speed;
	}

    public void autoSwitchShooterMode(){
        if(num % 2 == 1){
            shootMode = 1; //hood can move
          }
          else{
            shootMode = 0; //hood is fixed
          }
          num += 1;
    }

    public void setShootShooter(){
        shootState = ShooterControlState.SHOOT;
    }
    public void setPrepareShooter(){
        shootState = ShooterControlState.PREPARE_SHOOT;
        setSpeed(Constants.kFlywheelIdleVelocity);
    }

    public void setStopShooter(){
        shootState = ShooterControlState.STOP;
    }

    public void setInitShooter(){
        shootState = ShooterControlState.INIT;
    }

    public double readShooterSpeedFromShuffleBoard(){
        SmartDashboard.putNumber("Debug/Shooter/Shooter Speed", shooterSpeed.getDouble(1.0));
        return shooterSpeed.getDouble(1.0);
    }

    public double readHoodAngleFromShuffleBoard(){
        SmartDashboard.putNumber("Debug/Shooter/Hood Angle", hoodAngle.getDouble(20.0));
        return hoodAngle.getDouble(20.0);
    }

    public double[] GetMovingShotParams(double V, double phi_v, double phi_h, double vx, double vy){
        double phi_v_rad = Math.toRadians(phi_v);
        double phi_h_rad = Math.toRadians(phi_h);

        double theta_h_rad = Math.atan2((V*Math.cos(phi_v_rad)*Math.sin(phi_h_rad) - vy) ,
                                    (V*Math.cos(phi_v_rad)*Math.cos(phi_h_rad) - vx));
        double theta_v_rad = Math.atan(1/((V*Math.cos(phi_v_rad)*Math.cos(phi_h_rad) -
                                        vx) / (V*Math.sin(phi_v_rad)*Math.cos(theta_h_rad))));
        double vs = V*Math.sin(phi_v_rad)/Math.sin(theta_v_rad);

        double theta_h = Math.toDegrees(theta_h_rad);
        double theta_v = Math.toDegrees(theta_v_rad);

        double[] myShootParams = new double [3];
        myShootParams[0] = theta_h;
        myShootParams[1] = theta_v;
        myShootParams[2] = vs;

        return myShootParams;
    }

    public double getMoveOffset(){
        return MoveOffset;
    }

    public boolean whetherReadyToShoot(){
        return readyToShoot;
    }

    public enum ShooterControlState {
        STOP, INIT,PREPARE_SHOOT, SHOOT
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


    // for hood

    private void moveHoodMotor() {
        double targetPos = Conversions.degreesToTalon(HoodDesiredAngle, Constants.HOOD_GEAR_RATIO) + offset;
        HoodPeriodicIO.demand = (int)targetPos;
        mHoodmotor.set(ControlMode.MotionMagic, targetPos);
    }

    public void setHoodAngle(double targetAngle){
        // Preform Bounds checking between MAX and MIN range
        if (targetAngle < Constants.HOOD_MIN_ANGLE) {
            targetAngle = Constants.HOOD_MIN_ANGLE;
        }

        if (targetAngle > Constants.HOOD_MAX_ANGLE) {
            targetAngle = Constants.HOOD_MAX_ANGLE;
        }
        HoodDesiredAngle = targetAngle;        
    }
    
    public void setHoodStop(){
        HoodPeriodicIO.demand = 0;
        mHoodmotor.set(ControlMode.PercentOutput, 0);
    }
    
    public double getHoodAngle(){
        if (RobotBase.isSimulation()){
          return Conversions.talonToDegrees(HoodPeriodicIO.position - offset, Constants.HOOD_GEAR_RATIO);
        }else{
          return Conversions.talonToDegrees((int) mHoodmotor.getSelectedSensorPosition(0)- offset, Constants.HOOD_GEAR_RATIO);
        }
        
    }

    public void HoodWritePeriodicOutputs(){}

    public void HoodOutputTelemetry(){
        SmartDashboard.putNumber("Debug/Hood/Angle", getHoodAngle());
    }
    
    public void HoodReadPeriodicInputs() {
        if (RobotBase.isSimulation())
        {
            HoodPeriodicIO.position = (int)HoodPeriodicIO.demand;
        }else{
            HoodPeriodicIO.position = (int) mHoodmotor.getSelectedSensorPosition(0);  
        }
    }
    
    public void ZeroHood(){
        mHoodmotor.set(ControlMode.MotionMagic, offset);
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



    // new shooter
    /**
     * Turns the shooter feeder-wheel on/off
     *
     * @param shoot true to turn on, false to turn off
     */
    public void setFiring(boolean shoot) {
        if (shoot) {
            blockerState = BlockerControlState.BALLLOCKER_ON;
        } else {
            blockerState = BlockerControlState.BALLLOCKER_OFF;
        }
    }

    /**
     * Gets the state of the Feeder Wheel.
     * <p>
     * Can either be ON or OFF.
     */
    public BlockerControlState getBlockerState(){
        return blockerState;
    }

        /**
     * Sets Speed of Shooter FlywheelWheel.
     *
     * @param shooterSpeedRPM Desired Speed in RPM.
     */
    public void setSpeed(double shooterSpeedRPM) {
        this.ShooterDesiredSpeed = shooterSpeedRPM;
        if (ShooterDesiredSpeed == 0) {
            shootState = ShooterControlState.STOP;
        } 
    }

    @Override
    public void periodic() {

        moveHoodMotor();
        HoodReadPeriodicInputs();
        HoodOutputTelemetry();
        ShooterWritePeriodicOutputs();
        ShooterReadPeriodicInputs();
        ShooterOutputTelemetry();

    }


}
