package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

import javax.lang.model.util.ElementScanner6;

import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.lib.team1706.LinearInterpolationTable;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Shooter.ShooterControlState;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class AimManager extends SubsystemBase {
    private static AimManager instance = null;
    private AimManagerState currentState = AimManagerState.STOP;
    Turret turret = Turret.getInstance();
    Shooter shooter= Shooter.getInstance();
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    SwerveDriveTrain driveDrain = SwerveDriveTrain.getInstance();
    Hopper hooper = Hopper.getInstance();
    Intake intake = Intake.getInstance();
    ColorSensor colorsensor = ColorSensor.getInstance();
    private static LinearInterpolationTable m_hoodTable = ShooterConstants.kHoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;
    boolean enanbleTelemetry = false;
    double hoodAngle  = 0;
    double shooterRPM = 0;
    boolean wrongBall = false ;
    boolean topHasBall = false; 
    boolean hasBallShooting = false;
    double shotBallTime = Double.NEGATIVE_INFINITY;
    double shotWrongBallTime = Double.NEGATIVE_INFINITY;
    boolean hasWrongBallShooting = false;
    int shootBallCnt = 0;
    int shootWrongBallCnt = 0;
    AimManagerState saveShootStat = AimManagerState.STOP;
    private Timer m_timer = new Timer();
    ShuffleboardTab summaryTab = Shuffleboard.getTab("Summary");
    ShuffleboardTab AimTab = Shuffleboard.getTab("AimManager");
    NetworkTableEntry autoAimRangeEntry;
    private ShuffleboardTab shooterParaTab = Shuffleboard.getTab("Shoot Parameter");
    private NetworkTableEntry InputShooterSpeed =
        shooterParaTab.add("Shooter Speed", 1).getEntry();
    private NetworkTableEntry inputHoodAngle =
        shooterParaTab.add("Hood Angle", 1).getEntry();    
    
      /** Creates a new AutoAim. */
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20 / 2.54);  //TODO
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(0) + Units.inchesToMeters(55/2.54);  //TODO
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);  //TODO
            
    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(0.5/0.3048);  //TODO
    
    // PID constants should be tuned per robot
    final double LINEAR_P = 0.01;
    final double LINEAR_D = 0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
        
    final double ANGULAR_P = 0.02;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    Translation2d translation;
    


    public AimManager() {     
        addShuffleboardDebug();
        m_timer.reset();
        m_timer.start();
    }

    public static AimManager getInstance() {
        if (instance == null) {
            instance = new AimManager();
        }
        return instance;
    }

    public AimManagerState getAimManagerState() {
        return currentState;
    }

    public void setAimManagerState(AimManagerState state) {
        currentState = state;
    }
    public void startAimMoving() {
        currentState = AimManagerState.AIM_MOVING;
    }

    public void lockOnTarget() {
        currentState = AimManagerState.AIM_LOCKED;
    }

    public boolean isAimMoving() {
        return (currentState == AimManagerState.AIM_MOVING);
    }

    public boolean isAimLocked() {
        return (currentState == AimManagerState.AIM_LOCKED);
    }

    public boolean isStop() {
        return (currentState == AimManagerState.STOP);
    }

    public void Stop() {
        currentState = AimManagerState.STOP;
    }
    
    public boolean isTargetLocked() {
        return (Math.abs(limelight.Get_tx()) < Constants.TargetMinError
                && limelight.isTargetVisible());
    }

    public void writePeriodicOutputs() {
        double currentTime = m_timer.get();
        wrongBall = colorsensor.isWrongBall();
        topHasBall = hooper.isHasTopBall(); //TODO  must add top ball sensor
        //saveShootStat = currentState;

        if(topHasBall && wrongBall && !hasWrongBallShooting){
            saveShootStat = currentState;
            currentState = AimManagerState.AIM_WRONGBALL;
            shotWrongBallTime = currentTime;
            hasWrongBallShooting = true;
            doShooterEject();
        }else if (hasWrongBallShooting && currentTime < shotWrongBallTime + Constants.kShootOneBallTime) {
            doShooterEject();
        }else if(hasWrongBallShooting){
            hasWrongBallShooting = false;
            shotWrongBallTime = Double.NEGATIVE_INFINITY;       
            currentState = saveShootStat;
            shooter.setFiring(false);
            shootWrongBallCnt++;

        }
        else {
            hasWrongBallShooting = false;
            shotWrongBallTime = Double.NEGATIVE_INFINITY;
        }

        if (currentState == AimManagerState.STOP) {
            hasBallShooting = false; 
            shotBallTime = Double.NEGATIVE_INFINITY;
            //shooter.setShooterToStop();
            shooter.setFiring(false);
        } 
        if (currentState == AimManagerState.AIM_MOVING) {
            //shooter.setShooterToPrepare();
            DoAutoAim();
            if (isTargetLocked()) {
                hasBallShooting = false; 
                shotBallTime = Double.NEGATIVE_INFINITY;
                currentState = AimManagerState.AIM_LOCKED;
            } 
        }
        if (currentState == AimManagerState.AIM_LOCKED) {
            if (isTargetLocked()) {
                double dist = limelight.getRobotToTargetDistance();  //TODO
                shooterRPM = m_rpmTable.getOutput(dist);
                hoodAngle = m_hoodTable.getOutput(dist);
                //double dist = limelight.getDistance();
                //shooter.setShooterToShoot();
                shooter.setShooterSpeed(shooterRPM);
                //shooter.setHoodToOn();
                //double curspeed = shooter.getShooterSpeedRpm();
                shooter.setHoodAngle(hoodAngle);
                if(topHasBall && CanShot() && !hasBallShooting){
                    shotBallTime = currentTime;
                    hasBallShooting = true;
                    hooper.setHopperState(HopperState.ON);
                    shooter.setFiring(true);
                }else if(hasBallShooting && currentTime < shotBallTime + Constants.kShootOneBallTime){
                    hooper.setHopperState(HopperState.ON);
                    shooter.setFiring(true);                    
                }else if (hasBallShooting && topHasBall) {   
                    hasBallShooting = false; 
                    shotBallTime = Double.NEGATIVE_INFINITY;
                    hooper.setHopperState(HopperState.ON);
                    shooter.setFiring(true);//TODO how to support continious shoot!
                    shootBallCnt++;
                }else{
                    hasBallShooting = false;
                    shotBallTime = Double.NEGATIVE_INFINITY;
                    //hooper.setHopperState(HopperState.OFF);
                    shooter.setFiring(false);//TODO how to support continious shoot! 
                }
            }
            else{
                currentState = AimManagerState.AIM_MOVING;
            }
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Debug/AimManager/AimState", getAimManagerState().name());
        SmartDashboard.putString("Debug/AimManager/ShooterState", shooter.getShooterState().name()); 
        SmartDashboard.putBoolean("Debug/AimManager/hasBallShooting", hasBallShooting); 
        SmartDashboard.putBoolean("Debug/AimManager/topHasBall", topHasBall);        
        SmartDashboard.putBoolean("Debug/AimManager/wrongBall",wrongBall); 
        SmartDashboard.putBoolean("Debug/AimManager/hasWrongBallShooting",hasWrongBallShooting); 
        SmartDashboard.putNumber("Debug/AimManager/ShooterSpeedRPM", shooter.getShooterSpeedRpm()); 
        SmartDashboard.putString("Debug/AimManager/HoodState", shooter.getHoodState().name());  
        SmartDashboard.putNumber("Debug/AimManager/HoodAngle", shooter.getHoodAngle());
        SmartDashboard.putString("Debug/AimManager/BlockerState",shooter.getBlockerState().name());
        SmartDashboard.putString("Debug/AimManager/HooperState",hooper.getHopperState().name());
        SmartDashboard.putString("Debug/AimManager/IntakeState",intake.getWantedIntakeState().name());
        SmartDashboard.putString("Debug/AimManager/IntakeSolState",intake.getIntakeSolState().name());
    }

    public enum AimManagerState {
        STOP, AIM_LOCKED, AIM_MOVING, AIM_WRONGBALL
    }

    @Override
    public void periodic() {
        writePeriodicOutputs();
        if(enanbleTelemetry){
            outputTelemetry();       
        }
   

    }

    public void DoAutoAim(){
        double forwardSpeed;
        double rotationSpeed;
    
        double totalforwardSpeed;
        double totalrotationSpeed;
    
        double lastx = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftY());
        double lasty = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftX());
        double lastz = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getRightX());
    
        double llastx = - RobotContainer.m_swerve.deadband(lastx) * Constants.kMaxSpeed;
        double llasty = - RobotContainer.m_swerve.deadband(lasty) * Constants.kMaxSpeed;
        double llastz = lastz * Constants.kMaxOmega;
    
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        double target  = RobotContainer.m_limelight.Get_tv();
    
        if (target != 0.0f) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(RobotContainer.m_limelight.Get_ty()));
            SmartDashboard.putNumber("Debug/Auto/range", range);
            autoAimRangeEntry.setDouble(range);
            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            if(RobotContainer.m_limelight.Get_ty()<-1){
              forwardSpeed = 1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);;
            }
            else if(RobotContainer.m_limelight.Get_ty() > 1){
              forwardSpeed = -1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);
            }
            else{
              forwardSpeed = 0;
            }
    
    
            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -1.0 * turnController.calculate(RobotContainer.m_limelight.Get_tx(), 0);
          } else {
              // If we have no targets, stay still.
              forwardSpeed = 0;
              rotationSpeed = 0;
            } 
    
          totalforwardSpeed = /*forwardSpeed * Constants.kMaxSpeed*/ + llastx;
          totalrotationSpeed = rotationSpeed * Constants.kMaxOmega + llastz;
    
          translation = new Translation2d(totalforwardSpeed, llasty);
    
          // Use our forward/turn speeds to control the drivetrain
          //RobotContainer.m_swerve.Drive(forwardSpeed, 0, rotationSpeed, false);
    
          // Goal-Centric
          RobotContainer.m_swerve.Drive(translation, - totalrotationSpeed, true, false);//Use feedback control when auto aiming.
    
    }

    public boolean CanShot(){
        boolean isShooterReady = (Math.abs(shooter.getShooterSpeedRpm()- shooterRPM) < Constants.kShooterTolerance);
        boolean isHoodReady = (Math.abs(shooter.getHoodAngle() - hoodAngle) < Constants.HoodConstants.kHoodTolerance);

        return (isShooterReady && isHoodReady);
    }

    public void doShooterEject() {
        shooter.setShooterSpeed(Constants.SHOOTER_EJECT_SPEED);
        shooter.setHoodAngle(Constants.HOOD_EJECT_ANGLE);
        hooper.setHopperState(HopperState.ON);
        shooter.setFiring(true);
    }

    public double readShooterSpeedFromShuffleBoard(){
        //SmartDashboard.putNumber("Debug/Shooter/Shooter Speed", InputShooterSpeed.getDouble(1.0));
        return InputShooterSpeed.getDouble(1.0);
    }

    public double readHoodAngleFromShuffleBoard(){
        //SmartDashboard.putNumber("Debug/Shooter/Hood Angle", inputHoodAngle.getDouble(20.0));
        return inputHoodAngle.getDouble(20.0);
    }
    public void DebugShootParameter(){
        double inputShootSpeedRPM = 0;
        double inputHoodAngle = 0;
        inputShootSpeedRPM = readShooterSpeedFromShuffleBoard();
        inputHoodAngle = readHoodAngleFromShuffleBoard();
        shooter.setHoodAngle(inputHoodAngle);
        shooter.setShooterSpeed(inputShootSpeedRPM);
        shooter.setShooterToMannulShoot();
    }

    public boolean isHasBallShooting(){
        return hasBallShooting;
    }

    public boolean isHasWrongBallShooting(){
        return hasWrongBallShooting;
    }

    public int getShootBallCnt(){
        return shootBallCnt;
    }
    public int getShootWrongBallCnt(){
        return shootWrongBallCnt;
    }

    public void switchAimMode(){
        currentState = (currentState == AimManagerState.STOP) ? AimManagerState.AIM_MOVING:AimManagerState.STOP;
    }

    private void addShuffleboardDebug(){

        summaryTab.addString("AimState", () ->this.getAimManagerState().name())
            .withPosition(0, 0)
            .withSize(1, 1);
        summaryTab.addString("ShooterState", () ->shooter.getShooterState().name())
            .withPosition(0, 1)
            .withSize(1, 1);
        summaryTab.addNumber("ShooterSpeedRPM",() ->shooter.getShooterSpeedRpm())
            .withPosition(0, 2)
            .withSize(1, 1);         
        summaryTab.addString("HoodState", () ->shooter.getHoodState().name())
            .withPosition(0, 3)
            .withSize(1, 1);            
        summaryTab.addNumber("HoodAngle",() ->shooter.getHoodAngle())
            .withPosition(0, 2)
            .withSize(1, 1);    
        summaryTab.addString("BlockerState", () ->shooter.getBlockerState().name())
            .withPosition(2, 0)
            .withSize(1, 1);     
        summaryTab.addString("HooperState", () ->hooper.getHopperState().name())
            .withPosition(2, 1)
            .withSize(1, 1);     
        summaryTab.addString("IntakeState", () ->intake.getWantedIntakeState().name())
            .withPosition(2, 2)
            .withSize(1, 1);      
        summaryTab.addString("IntakeSolState", () ->intake.getIntakeSolState().name())
            .withPosition(2, 3)
            .withSize(1, 1);

        AimTab.addNumber("shootBallCnt",() ->this.getShootBallCnt())
            .withPosition(0, 0)
            .withSize(1, 1);                                        
        AimTab.addNumber("shootWrongBallCnt",() ->this.getShootWrongBallCnt())
            .withPosition(0, 1)
            .withSize(1, 1);
        AimTab.addBoolean("hasBottomBall",hooper::isHasBottomBall)
            .withPosition(0, 2)
            .withSize(1, 1);  
        AimTab.addBoolean("Wrongball",colorsensor::isWrongBall)
            .withPosition(0, 3)
            .withSize(1, 1);                      
        AimTab.addBoolean("hasBallShooting",() ->this.isHasBallShooting())
            .withPosition(1, 0)
            .withSize(1, 1);            

        AimTab.addBoolean("hasWrongBallShooting",() ->this.isHasWrongBallShooting())
            .withPosition(1, 1)
            .withSize(1, 1);          
        autoAimRangeEntry = AimTab.add("autoAim range", 0.0)
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry();        
    }

}
