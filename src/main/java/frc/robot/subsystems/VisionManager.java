package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.team503.util.Util;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.lib.team1678.math.Conversions;
import frc.robot.lib.team1690.Vector;
import frc.robot.lib.team1690.Vector3D;
import frc.robot.lib.team1706.FieldRelativeAccel;
import frc.robot.lib.team1706.FieldRelativeSpeed;
import frc.robot.lib.team1706.LinearInterpolationTable;
import frc.robot.lib.team2910.math.Vector2;
import frc.robot.lib.team2910.util.InterpolatingDouble;
import frc.robot.lib.team2910.util.InterpolatingTreeMap;

public class VisionManager extends SubsystemBase {
    private static int direction = 0;
    private static VisionManager instance = null;
    private VisionManagerState currentState = VisionManagerState.ZERO_TURRET;
    private int TurrentFindingTimes = 0;
    Turret turret = Turret.getInstance();
    Shooter shooter= Shooter.getInstance();
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    SwerveDriveTrain driveDrain = SwerveDriveTrain.getInstance();
    Hopper hooper = Hopper.getInstance();
    Intake intake = Intake.getInstance();
    private final Timer m_timer = new Timer();
    private double m_wrongBallTime;
    private double radialVelocity = 0;
    private double tangentialVelocity = 0;
    private double angleToGoal = 0;
    private double futureDist = 0;

    private static LinearInterpolationTable m_timeTable = ShooterConstants.kTimeTable;
    
    static InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();
    private int num = 1;
    private int shootMode = 1;//1 means table shoot ,0 means algorithm shoot
    private double MoveOffset = 0;
    //LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    double targetVelocity ;
    double targetHoodAngle ;
    double targetTurretAngle;

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
        

    public VisionManager() {
        ;
    }

    public static VisionManager getInstance() {
        if (instance == null) {
            instance = new VisionManager();
        }
        return instance;
    }

    public VisionManagerState getVisionManagerState() {
        return currentState;
    }

    public void setVisionManagerState(VisionManagerState state) {
        currentState = state;
    }

    public void ZeroTurret() {
        currentState = VisionManagerState.ZERO_TURRET;
    }

    public void startVisionMoving() {
        currentState = VisionManagerState.VISION_MOVING;
    }

    public void lockOnTarget() {
        currentState = VisionManagerState.VISION_LOCKED;
    }

    public boolean isVisionFinding() {
        return (currentState == VisionManagerState.VISION_FINDING);
    }

    public boolean isVisionMoving() {
        return (currentState == VisionManagerState.VISION_MOVING);
    }

    public boolean isVisionLocked() {
        return (currentState == VisionManagerState.VISION_LOCKED);
    }

    public boolean valueIsRange(double value, double minRange, double maxRange) {
        //return Math.abs(Math.max(minRange, value) ) - Math.abs(Math.min(value, maxRange)) < 0.0001;
        return Math.max(minRange, value) == Math.min(value, maxRange);
    }

    public boolean isVisionGoodRange(double angle) {
        return (valueIsRange(angle, Constants.TurretMinSoftLimit, Constants.TurretMaxSoftLimit)
                && limelight.isTargetVisible());
    }

    public boolean isStop() {
        return (currentState == VisionManagerState.STOP);
    }

    public void startVisionFinding() {
        currentState = VisionManagerState.VISION_FINDING;
        TurrentFindingTimes = 0;
        turret.On(true);
        shooter.setHoodToOn();
        shooter.setFiring(false);
    }

    public void Stop() {
        currentState = VisionManagerState.STOP;
    }

    public void startVisionMannul() {
        currentState = VisionManagerState.VISION_MANUAL;
    }
    
    public void readPeriodicInputs() {
        ;
    }

    public boolean isTargetLocked() {
        return (Math.abs(limelight.Get_tx()) < Constants.TargetMinError
                && limelight.isTargetVisible());
    }

    public void writePeriodicOutputs() {
        double desiredAngle = 0;

        if (currentState == VisionManagerState.VISION_MANUAL) {
            ;
        }

        if (currentState == VisionManagerState.ZERO_TURRET) {
            turret.ZeroTurret();
            shooter.setShooterToStop();
            shooter.setFiring(false);
        } 
        if (currentState == VisionManagerState.STOP) {
            turret.setTurretAngle(turret.getAngleDeg());
            shooter.setShooterToStop();
            shooter.setFiring(false);
        } 
        if ((currentState == VisionManagerState.VISION_FINDING)
              && ((TurrentFindingTimes % 5) == 0 )){ //TODO  TurrentFindingTimes will bei delete
            shooter.setFiring(false);
            desiredAngle = turret.getAngleDeg();
            if (direction == 0) {
                desiredAngle -= Constants.kTurretStep;
                desiredAngle = Math.max(desiredAngle, Constants.TurretMinSoftLimit);
                if (desiredAngle <= Constants.TurretMinSoftLimit) {
                    direction = 1;
                }
            } else {
                desiredAngle += Constants.kTurretStep;
                desiredAngle = Math.min(desiredAngle, Constants.TurretMaxSoftLimit);
                if (desiredAngle >= Constants.TurretMaxSoftLimit) {
                    direction = 0;
                }
            }
            turret.setTurretAngle(desiredAngle);
            if (isVisionGoodRange(turret.getAngleDeg() )) {
                currentState = VisionManagerState.VISION_MOVING;
            }
            shooter.setShooterToPrepare();
        } 
        if (currentState == VisionManagerState.VISION_MOVING) {
            desiredAngle = turret.getAngleDeg()- limelight.Get_tx();
            desiredAngle = Util.boundAngleNeg180to180Degrees(desiredAngle);
            turret.setTurretAngle(desiredAngle);
            if (isTargetLocked()) {
                currentState = VisionManagerState.VISION_LOCKED;
                shooter.setShooterToShoot();
            } else if (!isVisionGoodRange(turret.getAngleDeg()- limelight.Get_tx())) {
                currentState = VisionManagerState.VISION_FINDING;
                TurrentFindingTimes = 0;
                if (desiredAngle > 0) {
                    direction = 0;
                } else {
                    direction = 1;
                }
                shooter.setShooterToPrepare();
            }
            else{  // still is VISION_MOVING
                shooter.setShooterToPrepare();
            }
            
        }
        if (currentState == VisionManagerState.VISION_LOCKED) {
            shooter.setShooterToShoot();
            shooter.setHoodToOn();
            if (isVisionGoodRange(turret.getAngleDeg() - limelight.Get_tx())) {
                if(getShooterMode() == 1){
                    SetMovingShootParams();
                    //shootOnMoveOrbit();//Orbit's shotOnMove
                    desiredAngle = turret.getAngleDeg() + (Robot.isSimulation() ? 0 : getMoveOffset()); //TODO
                }else{
                    setFixedShootParams();
                    desiredAngle = turret.getAngleDeg();
                }
                turret.setTurretAngle(desiredAngle);
            }else {
                currentState = VisionManagerState.VISION_FINDING;
                TurrentFindingTimes = 0;
                if (desiredAngle < 0) {
                    direction = 0;
                } else {
                    direction = 1;
                }
            }
        }
        TurrentFindingTimes++;
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Debug/VisionManager/VisionState", getVisionManagerState().name());
        SmartDashboard.putNumber("Debug/VisionManager/ShooterMode", getShooterMode());
        SmartDashboard.putString("Debug/VisionManager/ShooterState", shooter.getShooterState().name()); 
        SmartDashboard.putNumber("Debug/VisionManager/ShooterSpeedRPM", shooter.getShooterSpeedRpm()); 
        SmartDashboard.putString("Debug/VisionManager/HoodState", shooter.getHoodState().name());  
        SmartDashboard.putNumber("Debug/VisionManager/HoodAngle", shooter.getHoodAngle());
        SmartDashboard.putString("Debug/VisionManager/TurretState", turret.getTurretState().name());
        SmartDashboard.putNumber("Debug/VisionManager/TurretAngle",turret.getAngleDeg());
        SmartDashboard.putString("Debug/VisionManager/BlockerState",shooter.getBlockerState().name());
        SmartDashboard.putString("Debug/VisionManager/HooperState",hooper.getHopperState().name());
        SmartDashboard.putString("Debug/VisionManager/OuttakeState",hooper.getOuttakeState().name());
        SmartDashboard.putString("Debug/VisionManager/IntakeState",intake.getWantedIntakeState().name());
        SmartDashboard.putString("Debug/VisionManager/IntakeSolState",intake.getIntakeSolState().name());
    }

    public enum VisionManagerState {
        ZERO_TURRET, STOP, VISION_LOCKED, VISION_MOVING, VISION_FINDING,VISION_MANUAL
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        if (limelight.getLightMode() == 3) {
            SmartShooter();
            writePeriodicOutputs();
        }
        outputTelemetry();       

    }

    public int getShooterMode(){
        return shootMode;
    }

    public void setFixedShootParams(){
        double  speed = Conversions.MPSToRPM( getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE),
                                     Constants.kFlyWheelCircumference);
        shooter.setSpeed(speed);
    }

    public void SetMovingShootParams(){
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(futureDist));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                Conversions.RPMToMPS(angleAndSpeed.y, Constants.kFlyWheelCircumference), // Shot Speed
                angleAndSpeed.x,                                                         // Hood Angle
                angleToGoal,         // Turret target angle (The same coordinate system with swerve)
                driveDrain.getFieldRelativeSpeed().vx,              // Swerve speed in X axis (field-oriented)
                driveDrain.getFieldRelativeSpeed().vy);             // Swerve speed in Y axis (field-oriented)
        targetVelocity = Conversions.MPSToRPM(mShotParams[2], Constants.kFlyWheelCircumference);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - driveDrain.GetHeading_Deg());
        MoveOffset = targetTurretAngle - turret.getAngleDeg();
        shooter.setSpeed(targetVelocity);
        shooter.setHoodAngle(targetHoodAngle);
    }

    public boolean isShooterCanShoot(){
        boolean result = false;
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(futureDist));
        double[] mShotParams = new double [3];
        mShotParams = GetMovingShotParams(
                Conversions.RPMToMPS(angleAndSpeed.y, Constants.kFlyWheelCircumference), // Shot Speed   
                angleAndSpeed.x,                                                         // Hood Angle
                angleToGoal,    // Turret Angle (The same coordinate system with swerve)
                driveDrain.getFieldRelativeSpeed().vx,              // Swerve speed in X axis (field-oriented)
                driveDrain.getFieldRelativeSpeed().vy);             // Swerve speed in Y axis (field-oriented)
        targetVelocity = Conversions.MPSToRPM(mShotParams[2],Constants.kFlyWheelCircumference);
        targetHoodAngle = mShotParams[1];
        targetTurretAngle = Util.boundAngleNeg180to180Degrees(
                        mShotParams[0] - driveDrain.GetHeading_Deg());
        result = (Math.abs(targetVelocity - shooter.getShooterSpeedRpm()) < 50)  //TODO  min error 
                 &&( Math.abs(targetTurretAngle - turret.getAngleDeg()) < 1.0)   //TODO  min error 
                 &&( Math.abs(targetHoodAngle - shooter.getHoodAngle()) < 1.0)   //TODO  min error 
                ; 
        return result;
    }

    public Vector3D calcBallVelocity() {
        float vel = (float)radialVelocity;
        float dist = (float)futureDist;
        //float vel = (float) -3.8;
        //float dist = (float) 5.0;

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

        final float tangentialRobotSpeed = (float)tangentialVelocity;
        final Vector tangentialRobotVel = Vector.fromAngleAndRadius(
                (float)angleToGoal + (float)Math.toRadians(90),
                tangentialRobotSpeed);
        final Vector3D tangentialRobotVel3D = new Vector3D(tangentialRobotVel.x, tangentialRobotVel.y, 0);

        final float angleToTargetWithoutRobotVel = (float)angleToGoal;

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
                .boundAngleNeg180to180Degrees(ballVelocity.getYaw() - driveDrain.GetHeading_Deg());
        MoveOffset = targetTurretAngle - turret.getAngleDeg();
        shooter.setSpeed(targetVelocity);
        shooter.setHoodAngle(targetHoodAngle);
    }

    public double getShooterLaunchVelocity(double shooterAngle) {
        double speed = 0;
        double d = Constants.CARGO_DIAMETER;
        double D = Constants.UPPER_HUB_DIAMETER;
        double g = 9.81;
        double H = Constants.LL_UPPER_HUB_HEIGHT;
        double h = Constants.SHOOTER_MOUNT_HEIGHT;
        double L = limelight.getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2; //getRobotToTargetDistance() + Constants.UPPER_HUB_DIAMETER / 2 ; 
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
    
    public void DebugShootParameter(){
        double inputShootSpeedRPM = 0;
        double inputHoodAngle = 0;
        inputShootSpeedRPM = readShooterSpeedFromShuffleBoard();
        inputHoodAngle = readHoodAngleFromShuffleBoard();
        startVisionMannul();
        shooter.setHoodAngle(inputHoodAngle);
        shooter.setShooterToMannulShoot();
        shooter.setSpeed(inputShootSpeedRPM);
    }

    public void doShooterEject() {
        startVisionMannul();
        shooter.setSpeed(Constants.SHOOTER_EJECT_SPEED);
        shooter.setHoodAngle(Constants.HOOD_EJECT_ANGLE);
        shooter.setFiring(true);
    }

    public void SmartShooter() {
        Translation2d drivePose = SwerveDriveTrain.getInstance().getPose().getTranslation();
        double currentTime = m_timer.get();
        boolean wrongBall = ColorSensor.getInstance().hasOpponentBall();
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

        double fixedShotTime = m_timeTable.getOutput(dist);//TODO
        double robotVelX = robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor;
        double robotVelY = robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor;

        double virtualDriveX = drivePose.getX()
                + fixedShotTime * robotVelX;      
        double virtualDriveY = drivePose.getY()
                + fixedShotTime * robotVelY;

        SmartDashboard.putNumber("Drive X", virtualDriveX);
        SmartDashboard.putNumber("Drive Y", virtualDriveY);

        Translation2d movingDriveLocation = new Translation2d(virtualDriveX,virtualDriveY);

        Translation2d toMovingDrive = movingDriveLocation.minus(target);

        futureDist = toMovingDrive.getDistance(new Translation2d());
        angleToGoal = Math.atan2(toMovingDrive.getY(), toMovingDrive.getX());
        radialVelocity = (robotVelY * Math.cos(angleToGoal) - (robotVelX * Math.sin(angleToGoal)));
        tangentialVelocity = (robotVelX * Math.sin(angleToGoal) + (robotVelY * Math.cos(angleToGoal)));
    }
}
