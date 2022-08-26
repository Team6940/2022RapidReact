package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.lib.team1706.LinearInterpolationTable;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.Constants.ShooterConstants;


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
    double hoodAngle  = 0;
    double shooterRPM = 0;
    
      /** Creates a new AutoAim. */
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20 / 2.54);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(0) + Units.inchesToMeters(55/2.54);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);
            
    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(0.5/0.3048);
    
    // PID constants should be tuned per robot
    final double LINEAR_P = 0.01;
    final double LINEAR_D = 0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
        
    final double ANGULAR_P = 0.02;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    Translation2d translation;
    

    public AimManager() {
        ;
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
        if (currentState == AimManagerState.STOP) {
            //shooter.setShooterToStop();
            //shooter.setFiring(false);
        } 
        if (currentState == AimManagerState.AIM_MOVING) {
            shooter.setShooterToPrepare();
            DoAutoAim();
            if (isTargetLocked()) {
                currentState = AimManagerState.AIM_LOCKED;
            } 
        }
        if (currentState == AimManagerState.AIM_LOCKED) {
            if (isTargetLocked()) {
                double dist = limelight.getRobotToTargetDistance();  //TODO
                //double dist = limelight.getDistance();
                shooter.setShooterSpeed(m_rpmTable.getOutput(dist));
                hooper.setHopperState(HopperState.ON);
                shooter.setHoodToOn();
                shooter.setHoodAngle(m_hoodTable.getOutput(dist));
                shooter.setShooterToShoot();

            }
            else{
                currentState = AimManagerState.AIM_MOVING;
            }
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Debug/AimManager/AimState", getAimManagerState().name());
        SmartDashboard.putString("Debug/AimManager/ShooterState", shooter.getShooterState().name()); 
        SmartDashboard.putNumber("Debug/AimManager/ShooterSpeedRPM", shooter.getShooterSpeedRpm()); 
        SmartDashboard.putString("Debug/AimManager/HoodState", shooter.getHoodState().name());  
        SmartDashboard.putNumber("Debug/AimManager/HoodAngle", shooter.getHoodAngle());
        SmartDashboard.putString("Debug/AimManager/BlockerState",shooter.getBlockerState().name());
        SmartDashboard.putString("Debug/AimManager/HooperState",hooper.getHopperState().name());
        SmartDashboard.putString("Debug/AimManager/IntakeState",intake.getWantedIntakeState().name());
        SmartDashboard.putString("Debug/AimManager/IntakeSolState",intake.getIntakeSolState().name());
    }

    public enum AimManagerState {
        STOP, AIM_LOCKED, AIM_MOVING
    }

    @Override
    public void periodic() {
        writePeriodicOutputs();
        outputTelemetry();       

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
}
