package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.FiveBallBottomMode;
import frc.robot.auto.modes.SixBallMode;
import frc.robot.auto.modes.TwoBallMode;
import frc.robot.auto.modes.FiveBallBottomMode2;
import frc.robot.auto.modes.SixBallMode2;
import frc.robot.auto.modes.TwoBallMode2;
import frc.robot.subsystems.SwerveDriveTrain;

public class AutonomousSelector {

    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("FiveBallBottom2", AutonomousMode.FIVE_BALL_BOTTOM2);
        autonomousModeChooser.addOption("SixBall2", AutonomousMode.SIX_BALL2);
        autonomousModeChooser.addOption("TwoBall2", AutonomousMode.TWO_BALL2);
        autonomousModeChooser.addOption("SixBall", AutonomousMode.FIVE_BALL_BOTTOM);
        autonomousModeChooser.addOption("SixBall", AutonomousMode.SIX_BALL);
        autonomousModeChooser.addOption("TwoBall", AutonomousMode.TWO_BALL);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public Command getCommand(SwerveDriveTrain s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case SIX_BALL2:
                return new SixBallMode2(s_Swerve);
            case FIVE_BALL_BOTTOM2:
                return new FiveBallBottomMode2(s_Swerve);
            case TWO_BALL2:
                return new TwoBallMode2(s_Swerve);                  
            case SIX_BALL:
                return new SixBallMode(s_Swerve);                
            case FIVE_BALL_BOTTOM:
                return new FiveBallBottomMode(s_Swerve);
            case TWO_BALL:
                return new TwoBallMode(s_Swerve);             
  
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    public AutonomousSelector() {
    }

    private enum AutonomousMode {
        FIVE_BALL_BOTTOM2,
        SIX_BALL2,
        TWO_BALL2,  
        FIVE_BALL_BOTTOM,
        SIX_BALL,
        TWO_BALL   
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}
