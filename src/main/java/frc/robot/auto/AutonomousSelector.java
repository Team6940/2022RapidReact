package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.FiveBallBottomMode;
import frc.robot.auto.modes.SixBallMode;
import frc.robot.auto.modes.TwoBallMode;
import frc.robot.subsystems.SwerveDriveTrain;

public class AutonomousSelector {

    private static SendableChooser<Rotation2d> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("FiveBallBottom", AutonomousMode.FIVE_BALL_BOTTOM);
        autonomousModeChooser.addOption("SixBall", AutonomousMode.SIX_BALL);
        autonomousModeChooser.addOption("TwoBall", AutonomousMode.TWO_BALL);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public Command getCommand(SwerveDriveTrain s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case FIVE_BALL_BOTTOM:
                return new FiveBallBottomMode(s_Swerve);

            case SIX_BALL:
                return new SixBallMode(s_Swerve);

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
        FIVE_BALL_BOTTOM,
        SIX_BALL,
        TWO_BALL,
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}
