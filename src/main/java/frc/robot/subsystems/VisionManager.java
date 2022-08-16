package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.team503.util.Util;

public class VisionManager extends SubsystemBase {

    private static final double maxSoftLimit = 100; // 度数 //TODO
    private static final double minSoftLimit = -193; // TODO
    private static int direction = 0;
    private static VisionManager instance = null;
    PeriodicIO periodicIO = new PeriodicIO();
    private VisionManagerState currentState = VisionManagerState.ZERO_TURRET;
    private int TurrentFindingTimes = 0;

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
        return (valueIsRange(angle, minSoftLimit, maxSoftLimit)
                && LimelightSubsystem.getInstance().isTargetVisible());
    }

    public boolean isStop() {
        return (currentState == VisionManagerState.STOP);
    }

    public void startVisionFinding() {
        currentState = VisionManagerState.VISION_FINDING;
        TurrentFindingTimes = 0;
        Turret.getInstance().On(true);
    }

    public void Stop() {
        currentState = VisionManagerState.STOP;
    }

    public void readPeriodicInputs() {
        ;
    }

    public boolean isTargetLocked() {
        return (Math.abs(LimelightSubsystem.getInstance().Get_tx()) < Constants.TargetMinError
                && LimelightSubsystem.getInstance().isTargetVisible());
    }

    public void writePeriodicOutputs() {
        double desiredAngle = 0;
        Turret  turret = Turret.getInstance();
        Shooter shooter = Shooter.getInstance();
        LimelightSubsystem limelight = LimelightSubsystem.getInstance();

        if (currentState == VisionManagerState.ZERO_TURRET) {
            turret.ZeroTurret();
            shooter.setStopShooter();
        } 
        if (currentState == VisionManagerState.STOP) {
            turret.setTurretAngle(turret.getAngleDeg());
            shooter.setStopShooter();
        } 
        if ((currentState == VisionManagerState.VISION_FINDING)
              && ((TurrentFindingTimes % 5) == 0 )){ //TODO  TurrentFindingTimes will bei delete
            desiredAngle = turret.getAngleDeg();
            if (direction == 0) {
                desiredAngle -= Constants.kTurretStep;
                desiredAngle = Math.max(desiredAngle, minSoftLimit);
                if (desiredAngle <= minSoftLimit) {
                    direction = 1;
                }
            } else {
                desiredAngle += Constants.kTurretStep;
                desiredAngle = Math.min(desiredAngle, maxSoftLimit);
                if (desiredAngle >= maxSoftLimit) {
                    direction = 0;
                }
            }
            turret.setTurretAngle(desiredAngle);
            if (isVisionGoodRange(limelight.Get_tx() + turret.getAngleDeg())) {
                currentState = VisionManagerState.VISION_MOVING;
            }
            shooter.setPrepareShooter();
        } 
        if (currentState == VisionManagerState.VISION_MOVING) {
            desiredAngle = limelight.Get_tx() + turret.getAngleDeg();
            desiredAngle = Util.boundAngleNeg180to180Degrees(desiredAngle);
            turret.setTurretAngle(desiredAngle);
            if (isTargetLocked()) {
                currentState = VisionManagerState.VISION_LOCKED;
                shooter.setShootShooter();
            } else if (!isVisionGoodRange(limelight.Get_tx() + turret.getAngleDeg())) {
                currentState = VisionManagerState.VISION_FINDING;
                TurrentFindingTimes = 0;
                if (desiredAngle > 0) {
                    direction = 0;
                } else {
                    direction = 1;
                }
                shooter.setPrepareShooter();
            }
            else{  // still is VISION_MOVING
                shooter.setPrepareShooter();
            }
            
        }
        if (currentState == VisionManagerState.VISION_LOCKED) {
            shooter.setShootShooter();
            if (isVisionGoodRange(limelight.Get_tx() + turret.getAngleDeg())) {
                if(shooter.getShooterMode() == 1){
                    shooter.SetMovingShootParams();
                    //Shooter2.getInstance().shootOnMoveOrbit();//Orbit's shotOnMove
                    desiredAngle = turret.getAngleDeg()  /*+ Shooter2.getInstance().getMoveOffset()*/;  //TODO
                }else{
                    shooter.setFixedShootParams();
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
        SmartDashboard.putString("Debug/VisionManager/State", getVisionManagerState().name());
    }

    public enum VisionManagerState {
        ZERO_TURRET, VISION_LOCKED, VISION_MOVING, VISION_FINDING, STOP
    }

    public static class PeriodicIO {
        // Inputs
        public double position;
        public int velocity;
        public double voltage;
        public double current;

        // Outputs
        public double demand;
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        if(LimelightSubsystem.getInstance().getLightMode() == 3){
            VisionManager.getInstance().writePeriodicOutputs();
        }
        VisionManager.getInstance().outputTelemetry();       

    }
    
}
