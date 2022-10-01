package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.lib.team1678.math.Conversions;
import frc.robot.lib.team1706.FieldRelativeAccel;
import frc.robot.lib.team1706.FieldRelativeSpeed;
import frc.robot.lib.team1706.LinearInterpolationTable;
import frc.robot.lib.team503.util.Util;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Shooter.ShooterControlState;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
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
    FeedManager feeder = FeedManager.getInstance();
    ColorSensor colorsensor = ColorSensor.getInstance();
    private static LinearInterpolationTable m_hoodTable = ShooterConstants.kHoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;
    boolean enanbleTelemetry = true;
    boolean wrongBall = false ;
    boolean topHasBall = false; 
    boolean bottomHasBall = false;
    boolean startBallShooting = false;
    boolean startForceBallShooting = false;
    double shotBallTime = Double.NEGATIVE_INFINITY;
    double WaitBallTime = Double.NEGATIVE_INFINITY;
    double shotWrongBallTime = Double.NEGATIVE_INFINITY;
    boolean hasWrongBallShooting = false;
    int shootBallCnt = 0;
    int shootWrongBallCnt = 0;
    double lastShooterRPM = 0;
    double lastHoodAngle = 0;
    AimManagerState saveShootStat = AimManagerState.STOP;
    private Timer m_timer = new Timer();
    ShuffleboardTab summaryTab = Shuffleboard.getTab("Summary");
    ShuffleboardTab AimTab = Shuffleboard.getTab("AimManager");
    NetworkTableEntry autoAimRangeEntry, DistanceEntry,RPMDiffEntry,HoodAngleDiffEntry,lookupTableRPM;
    private ShuffleboardTab shooterParaTab = Shuffleboard.getTab("Shoot Parameter");
    private NetworkTableEntry InputShooterSpeed =
        shooterParaTab.add("Shooter Speed", 1).getEntry();
    private NetworkTableEntry inputHoodAngle =
        shooterParaTab.add("Hood Angle", 1).getEntry();    
    
    private NetworkTableEntry inputDistance =
        shooterParaTab.add("inputDistance", 1).getEntry();    
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

    public double rotationSpeed = 0;

    Translation2d translation;
    
    private double m_wrongBallTime;
    private double radialVelocity = 0;
    private double tangentialVelocity = 0;
    private double angleToGoal = 0;
    private double futureDist = 0;
    public double tXoffset = 0;

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

    public void startAimShoot() {
        currentState = AimManagerState.AIM_SHOOT;
        //feeder.setAimFeed();
        //feeder.setAutoFeed();
    }

    public boolean isAimShoot(){
          return currentState == AimManagerState.AIM_SHOOT;
    }

    public void startAimForce() {
        //feeder.setAimFeed();;
        currentState = AimManagerState.AIM_FORCE;
    }    

    public void DoShootForve(){

    }

    public boolean isStop() {
        return (currentState == AimManagerState.STOP);
    }

    public void Stop() {
        currentState = AimManagerState.STOP;
    }
    
    public boolean isTargetLocked() {
        return (Math.abs(limelight.Get_tx()) < TurretConstants.TargetMinError
                && limelight.isTargetVisible());
    }

    public void writePeriodicOutputs() {
        double currentTime = m_timer.get();
        wrongBall = colorsensor.isWrongBall();
        //wrongBall = false;
        topHasBall = hooper.isHasTopBall();
        //topHasBall = true;
        bottomHasBall = hooper.isHasBottomBall();

        if (topHasBall && wrongBall) {
            //saveShootStat = currentState;
            feeder.setAimFeed();
            currentState = AimManagerState.AIM_WRONGBALL;
        }

        if (currentState == AimManagerState.AIM_WRONGBALL) {
            if (!hasWrongBallShooting) {
                shotWrongBallTime = currentTime;
                doShooterEject();
                if (CanShot()) {
                    hasWrongBallShooting = true;    
                    shooter.setFiring(true);
                }
            } else if (hasWrongBallShooting
                    && currentTime < shotWrongBallTime + ShooterConstants.kShootOneWrongBallTime) {
                doShooterEject();
                if (CanShot()) {
                    shooter.setFiring(true);
                }
            } else {
                hasWrongBallShooting = false;
                shotWrongBallTime = Double.NEGATIVE_INFINITY;
                currentState = AimManagerState.STOP;
                shootWrongBallCnt++;
                shooter.setFiring(false);
                shooter.setShooterToStop();
                hooper.setHopperState(HopperState.OFF);
            }
        }

        if (currentState == AimManagerState.STOP) {
            startBallShooting = false;
            shotBallTime = Double.NEGATIVE_INFINITY;
            //feeder.setAutoFeed();
            //shooter.setShooterToStop();
            //shooter.setFiring(false);
        }

        if (currentState == AimManagerState.AIM_SHOOT) {
            if (/*topHasBall &&*/ isTargetLocked()) {
                double dist = limelight.getRobotToTargetDistance(); 
                //double dist = limelight.getDistance();
                double speed = m_rpmTable.getOutput(dist);
                double angle = m_hoodTable.getOutput(dist);
                if( Math.abs(speed - lastShooterRPM) < ShooterConstants.kPreventShooterOscilliationRPM ){
                    speed = lastShooterRPM;
                }else{
                    lastShooterRPM = speed;
                }
                if( Math.abs(angle - lastHoodAngle) < Constants.HoodConstants.kHoodTolerance){
                    angle = lastHoodAngle;
                }else{
                    lastHoodAngle = angle;
                }
                shooter.setShooterSpeed(speed);
                shooter.setHoodAngle(angle);
                //SetMovingShootParams();
/*
                if (!startBallShooting) {
                    shotBallTime = currentTime;
                    hooper.setHopperState(HopperState.ON);
                    if (CanShot()) {
                        shooter.setFiring(true);
                        startBallShooting = true;
                    }
                } else if (startBallShooting
                        && currentTime < shotBallTime + ShooterConstants.kShootOneBallTime) {
                    hooper.setHopperState(HopperState.ON);
                    if (CanShot() ) { //&& currentTime > shotBallTime + ShooterConstants.kWaitBallTime
                        shooter.setFiring(true);
                    }
                } else {
                    startBallShooting = false;
                    shotBallTime = Double.NEGATIVE_INFINITY;
                    shooter.setFiring(false);
                    //shooter.setShooterToStop();
                    //hooper.setHopperState(HopperState.OFF);
                    shootBallCnt++;
                }
*/                
            } else if (topHasBall){
                startBallShooting = false;
                shotBallTime = Double.NEGATIVE_INFINITY;
//                shooter.setFiring(false); 
                shooter.setShooterToPrepare();
            }
            else{
                shooter.setShooterToStop();
            }
/*            else {
                currentState = AimManagerState.STOP;
                startBallShooting = false;
                shotBallTime = Double.NEGATIVE_INFINITY;
//                shooter.setFiring(false);
                shooter.setShooterToStop();
            }*/
        }

        if (currentState == AimManagerState.AIM_FORCE) {
            if (!startForceBallShooting) {
                shotBallTime = currentTime;
                //startForceBallShooting = true;
                if (CanShot()) {
                    ///shooter.setFiring(true);
                    startForceBallShooting = true;
                    //hooper.setHopperState(HopperState.ON);
                }
            } else if (startForceBallShooting
                    && currentTime < shotBallTime + ShooterConstants.kShootTestTime) {
                //hooper.setHopperState(HopperState.ON);
                if (CanShot()) {
                    //shooter.setFiring(true);
                }
            } else {
                startForceBallShooting = false;
                shotBallTime = Double.NEGATIVE_INFINITY;
                //shooter.setFiring(false);
                shooter.setShooterToStop();
                //hooper.setHopperState(HopperState.OFF);
                shootBallCnt++;
                currentState = AimManagerState.STOP;
            }
        }
    }        
    
    public void StopAll() {
        shooter.setShooterToStop();
        //shooter.setFiring(false);
        //hooper.setHopperState(HopperState.OFF);
    }

    public void outputTelemetry() {
        /*SmartDashboard.putString("Debug/AimManager/AimState", getAimManagerState().name());
        SmartDashboard.putString("Debug/AimManager/ShooterState", shooter.getShooterState().name()); 
        SmartDashboard.putBoolean("Debug/AimManager/hasBallShooting", startBallShooting); 
        SmartDashboard.putBoolean("Debug/AimManager/BottomHasBall", bottomHasBall);        
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
        SmartDashboard.putString("Debug/AimManager/feederState", feeder.getFeederState().name());*/

        SmartDashboard.putBoolean("CanShot",this.CanShot()); 
        
    }

    public enum AimManagerState {
        STOP, AIM_WRONGBALL,AIM_SHOOT,AIM_FORCE
    }

    @Override
    public void periodic() {
        //SmartShooter();
        writePeriodicOutputs();
        if(enanbleTelemetry){
            outputTelemetry();       
        }
    }

    public boolean CanShot(){
        double rpmDiff ,angeDiff;
        boolean isShooterReady,isHoodReady;

        rpmDiff = Math.abs(shooter.getShooterSpeedRpm()- shooter.getDesiredShooterSpeedRpm()) ;
        angeDiff = Math.abs(shooter.getHoodAngle() - shooter.getDesiredHoodAngle());
        RPMDiffEntry.setDouble(rpmDiff);
        HoodAngleDiffEntry.setDouble(angeDiff);
        isShooterReady = rpmDiff < ShooterConstants.kShooterTolerance;
        isHoodReady = angeDiff < HoodConstants.kHoodTolerance;

        return (isShooterReady && isHoodReady);
    }

    public void doShooterEject() {
        shooter.setShooterSpeed(ShooterConstants.SHOOTER_EJECT_SPEED);
        shooter.setHoodAngle(HoodConstants.HOOD_EJECT_ANGLE);
        hooper.setHopperState(HopperState.ON);
        //SwerveDriveTrain.getInstance().Drive(new Translation2d(0, 0), 1, true, true);
    }

    public double readShooterSpeedFromShuffleBoard(){
        //SmartDashboard.putNumber("Debug/Shooter/Shooter Speed", InputShooterSpeed.getDouble(1.0));
        return InputShooterSpeed.getDouble(1.0);
    }

    public double readHoodAngleFromShuffleBoard(){
        //SmartDashboard.putNumber("Debug/Shooter/Hood Angle", inputHoodAngle.getDouble(20.0));
        return inputHoodAngle.getDouble(20.0);
    }
    public double readDistanceromShuffleBoard(){
        //SmartDashboard.putNumber("Debug/Shooter/Hood Angle", inputDistance.getDouble(20.0));
        return inputDistance.getDouble(20.0);
    }
    
    public void DebugShootParameter(){
        double inputShootSpeedRPM = 0;
        double inputHoodAngle = 0;
        double inputDist = 0;
        double calcRPM = 0;
        inputShootSpeedRPM = readShooterSpeedFromShuffleBoard();
        inputHoodAngle = readHoodAngleFromShuffleBoard();
        inputDist = readDistanceromShuffleBoard();
        calcRPM = m_rpmTable.getOutput(inputDist);
        lookupTableRPM.setDouble(calcRPM);
        shooter.setHoodAngle(inputHoodAngle);
        shooter.setShooterSpeed(inputShootSpeedRPM);
        startAimForce();
    }

    public boolean isHasBallShooting(){
        return startBallShooting;
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
    
    public double getRotationSpeed(){
        return rotationSpeed;
    }

    public void SetMovingShootParams() {
        double angle = m_hoodTable.getOutput(futureDist);
        double speed = m_rpmTable.getOutput(futureDist);
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                Conversions.RPMToMPS(speed, ShooterConstants.kFlyWheelCircumference), // Shot Speed
                angle,                                                         // Hood Angle
                angleToGoal,         // Turret target angle (The same coordinate system with swerve)
                driveDrain.getFieldRelativeSpeed().vx,              // Swerve speed in X axis (field-oriented)
                driveDrain.getFieldRelativeSpeed().vy);             // Swerve speed in Y axis (field-oriented)
        double targetVelocity = Conversions.MPSToRPM(mShotParams[2], ShooterConstants.kFlyWheelCircumference);
        double targetHoodAngle = mShotParams[1];
        if( Math.abs(targetVelocity - lastShooterRPM) < ShooterConstants.kShooterTolerance ){
            targetVelocity = lastShooterRPM;
        }else{
            lastShooterRPM = targetVelocity;
        }
        if( Math.abs(targetHoodAngle - lastHoodAngle) < Constants.HoodConstants.kHoodTolerance){
            targetHoodAngle = lastHoodAngle;
        }else{
            lastHoodAngle = targetHoodAngle;
        }
        tXoffset = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - driveDrain.GetHeading_Deg());
        //double MoveOffset = targetTurretAngle - turret.getTurretAngleDeg();
        shooter.setShooterSpeed(targetVelocity);
        shooter.setHoodAngle(targetHoodAngle);
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

    public void SmartShooter() {
        Translation2d drivePose = SwerveDriveTrain.getInstance().getPose().getTranslation();
        double currentTime = m_timer.get();
        boolean wrongBall = colorsensor.isWrongBall();
        if (wrongBall) {
            m_wrongBallTime = currentTime;
        }
        SmartDashboard.putNumber("Current Time", currentTime);

        SmartDashboard.putBoolean("Wrong Ball", wrongBall);

        SmartDashboard.putBoolean("Shooter Running", true);

        FieldRelativeSpeed robotVel = SwerveDriveTrain.getInstance().getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = SwerveDriveTrain.getInstance().getFieldRelativeAccel();

        Translation2d target = GoalConstants.kGoalLocation;

        if (currentTime <= m_wrongBallTime + 0.100) {
            target = GoalConstants.kWrongBallGoal;
        }

        Translation2d robotToGoal = target.minus(drivePose);
        double dist = robotToGoal.getDistance(new Translation2d());

        SmartDashboard.putNumber("Calculated (in)", dist);

        double fixedShotTime = ShooterConstants.kShootOneBallTime;//TODO
        double robotVelX = robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor;
        double robotVelY = robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor;

        double virtualDriveX = drivePose.getX()
                + fixedShotTime * robotVelX;
        double virtualDriveY = drivePose.getY()
                + fixedShotTime * robotVelY;

        SmartDashboard.putNumber("Drive X", virtualDriveX);
        SmartDashboard.putNumber("Drive Y", virtualDriveY);

        Translation2d movingDriveLocation = new Translation2d(virtualDriveX, virtualDriveY);

        Translation2d toMovingDrive = movingDriveLocation.minus(target);

        futureDist = toMovingDrive.getDistance(new Translation2d());
        angleToGoal = Math.atan2(toMovingDrive.getY(), toMovingDrive.getX());
        radialVelocity = (robotVelY * Math.cos(angleToGoal) - (robotVelX * Math.sin(angleToGoal)));
        tangentialVelocity = (robotVelX * Math.sin(angleToGoal) + (robotVelY * Math.cos(angleToGoal)));
    }
    
    public double getTxOffset() {
        return tXoffset;
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
            .withPosition(1, 1)
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
        summaryTab.addString("FeederState", () ->feeder.getFeederState().name())
            .withPosition(2, 4)
                .withSize(1, 1);
    
        summaryTab.addNumber("ShooterVelocityUnit", () ->shooter.getShooterVelocityUnit())
            .withPosition(3, 0)
                .withSize(1, 1);
            
        //summaryTab.addBoolean("CanShot", () -> this.CanShot())
        //    .withPosition(3, 2)
        //    .withSize(1, 1);

        AimTab.addNumber("distance", () -> this.limelight.getRobotToTargetDistance())
            .withPosition(2,0)
            .withSize(1, 1);   

        AimTab.addBoolean("targetLocked", () -> this.isTargetLocked())
            .withPosition(2,1)
            .withSize(1, 1);             
        RPMDiffEntry = AimTab.add("RPM diff", 0.0)
            .withPosition(2, 2)
            .withSize(1, 1)
            .getEntry();     
        HoodAngleDiffEntry = AimTab.add("HoodAngel diff", 0.0)
            .withPosition(2, 3)
            .withSize(1, 1)
            .getEntry();              
        AimTab.addNumber("shootBallCnt",() ->this.getShootBallCnt())
            .withPosition(0, 0)
            .withSize(1, 1);                                        
        AimTab.addNumber("shootWrongBallCnt",() ->this.getShootWrongBallCnt())
            .withPosition(0, 1)
            .withSize(1, 1);
        AimTab.addBoolean("hasBottomBall",hooper::isHasBottomBall)
            .withPosition(0, 2)
                .withSize(1, 1);
        AimTab.addBoolean("hasTopBall",hooper::isHasTopBall)
            .withPosition(0, 3)
            .withSize(1, 1);  
        AimTab.addBoolean("Wrongball",colorsensor::isWrongBall)
            .withPosition(0, 4)
            .withSize(1, 1);                      
        /*AimTab.addBoolean("hasBallShooting",() ->this.isHasBallShooting())
            .withPosition(0, 5)
            .withSize(1, 1);       */     

        /*AimTab.addBoolean("hasWrongBallShooting",() ->this.isHasWrongBallShooting())
            .withPosition(0, 6)
            .withSize(1, 1);*/

        AimTab.addNumber("limelight tx", () -> limelight.Get_tx())
            .withPosition(3,0)
            .withSize(1, 1);
        AimTab.addNumber("rotation Speed", () -> this.getRotationSpeed())
            .withPosition(3,1)
            .withSize(1, 1);
        autoAimRangeEntry = AimTab.add("autoAim range", 0.0)
            .withPosition(3, 2)
            .withSize(1, 1)
            .getEntry();       
        lookupTableRPM = AimTab.add("LookupTable RPM", 0.0)
            .withPosition(3, 3)
            .withSize(1, 1)
            .getEntry();        
                   


    
        
  
    }

}
