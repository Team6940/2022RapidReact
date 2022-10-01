package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper.HopperState;

public final class FeedManager extends SubsystemBase  {

    private static FeedManager instance = null;

    private Timer m_timer = new Timer();
    double topBallHoldTime = 0;
    double noneBallTime = 0;
    double shotBallTime = Double.NEGATIVE_INFINITY;

    public static FeedManager getInstance() {
        if (instance == null){
            instance = new FeedManager();
        }
        return instance;
    }

    public enum FeedManagerState {
        AUTO_FEED, NORMAL_FEED
    }
    FeedManagerState fmState = FeedManagerState.NORMAL_FEED;


    private FeedManager() {
        m_timer.reset();
        m_timer.start();
        fmState = FeedManagerState.NORMAL_FEED;
    }

    public void setAutoFeed(){
        fmState = FeedManagerState.AUTO_FEED;
    }

    public void setAimFeed(){
        fmState = FeedManagerState.NORMAL_FEED;
    }

    public boolean isNormalFeed(){
        return  fmState ==  FeedManagerState.NORMAL_FEED;
    }

    public boolean isAutoFeed(){
        return  fmState ==  FeedManagerState.AUTO_FEED;
    }

    public FeedManagerState getFeederState(){
        return fmState;
    }

    private void writePeriodicOutputs(){
        Intake intake = Intake.getInstance();
        Hopper hooper = Hopper.getInstance();
        Shooter shooter = Shooter.getInstance();
        AimManager aim = AimManager.getInstance();
        double currentTime = m_timer.get();
        boolean topHasBall = hooper.isHasTopBall();
        boolean bottomHasBall = hooper.isHasBottomBall();
        boolean intakeOn = intake.isIntakerOn();
        
        // top = true, intake = off, bottom =false ,3s : stop
        
        // intake=off && bottom =false & top=false ,2s: stop
        if (fmState == FeedManagerState.AUTO_FEED) {
            //shotBallTime = currentTime;
            if (aim.isAimShoot() && aim.isTargetLocked() && aim.CanShot()) {
                //if (currentTime < shotBallTime + ShooterConstants.kShootOneBallTime) {
                //    hooper.setHopperState(HopperState.ON);
                //    shooter.setFiring(true);
                //}
                hooper.setHopperState(HopperState.ON);
                //shooter.setFiring(true);
                return;
            }else{
                //shooter.setFiring(false);
            }

            if( !intakeOn && !bottomHasBall && !topHasBall){
                if( (currentTime - noneBallTime) > 2.0 ){
                    hooper.setHopperState(HopperState.OFF);
                }else{
                    hooper.setHopperState(HopperState.ON);
                }
            }else{
                noneBallTime = currentTime;
            }
            
            // top = true, intake = off, bottom =false ,3s : stop
            if (topHasBall && !intakeOn && !bottomHasBall) {
                if( (currentTime - topBallHoldTime) > 3.0 ){
                    hooper.setHopperState(HopperState.OFF);
                    fmState = FeedManagerState.NORMAL_FEED;
                }else{
                    hooper.setHopperState(HopperState.ON);
                }
                //return;
            }else{
                topBallHoldTime = currentTime;            
            }

            if (intakeOn || bottomHasBall) {
                hooper.setHopperState(HopperState.ON);
                //return ;
            }  
        }

        if(fmState == FeedManagerState.NORMAL_FEED) {
            ;
        }
    }
    
    @Override
    public void periodic() {
        writePeriodicOutputs();
    }

    public void switchFeederMode() {
        fmState =  (fmState == FeedManagerState.NORMAL_FEED)
                        ?FeedManagerState.AUTO_FEED : FeedManagerState.NORMAL_FEED;
    }
}
