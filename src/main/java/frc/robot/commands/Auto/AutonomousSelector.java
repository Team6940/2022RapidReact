package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto.PathPlanner.PathPlannerWithTwo;
import frc.robot.subsystems.SwerveDriveTrain;

public class AutonomousSelector {

    private static SendableChooser<Rotation2d> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("1PathPlannerWithTwo", AutonomousMode.PATH_PLANNER_WITH_TWO);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public Command getCommand(SwerveDriveTrain s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case PATH_PLANNER_WITH_TWO:
                return new PathPlannerWithTwo(s_Swerve);

            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    public AutonomousSelector() {
    }

    private enum AutonomousMode {
        PATH_PLANNER_WITH_TWO,
        BOUNCE_AUTO,
        EXAMPLE_AUTO,
        COUNT321_AUTO,
        SWERVE1126,
        CURVE_PATH,
        PATH_PLANNER_STRAIGHT,
        CURVE_LINE_WAYPOINT,
        PATH_PLANNER_BOUNCE,
        STRAIGHT_LINE_MODE,
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}
