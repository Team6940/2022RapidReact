package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.commands.Autos.PathPlanner.PathPlannerBounce;
import frc.robot.commands.Autos.PathPlanner.PathPlannerStraight;
import frc.robot.commands.Autos.PathPlanner.PathPlannerWithTwo;
import frc.robot.commands.Autos.PathWeaver.BouncePathAuto;
import frc.robot.commands.Autos.PathWeaver.Count321Path;
import frc.robot.commands.Autos.PathWeaver.CurveLineWayPoint;
import frc.robot.commands.Autos.PathWeaver.CurvePath;
import frc.robot.commands.Autos.PathWeaver.Swerve1126;
import frc.robot.commands.Autos.WayPointsControl.StraightLineMode;

public class AutonomousSelector {

    private static SendableChooser<Rotation2d> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("1PathPlannerWithTwo", AutonomousMode.PATH_PLANNER_WITH_TWO);
        autonomousModeChooser.addOption("Bounce Path", AutonomousMode.BOUNCE_AUTO);
        autonomousModeChooser.addOption("Straight Line", AutonomousMode.STRAIGHT_LINE_MODE);

        autonomousModeChooser.addOption("Count321Path", AutonomousMode.COUNT321_AUTO);
        autonomousModeChooser.addOption("Swerve1126", AutonomousMode.SWERVE1126);

        autonomousModeChooser.addOption("CurvePath", AutonomousMode.CURVE_PATH);
        autonomousModeChooser.addOption("PathPlannerStraight", AutonomousMode.PATH_PLANNER_STRAIGHT);
        
        autonomousModeChooser.addOption("CurveLineWayPoint", AutonomousMode.CURVE_LINE_WAYPOINT);
        autonomousModeChooser.addOption("PathPlannerBounce", AutonomousMode.PATH_PLANNER_BOUNCE);
        autonomousModeChooser.addOption("StraightLineMode", AutonomousMode.STRAIGHT_LINE_MODE);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public Command getCommand(SwerveDriveTrain s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case PATH_PLANNER_WITH_TWO:
                return new PathPlannerWithTwo(s_Swerve);

            case BOUNCE_AUTO:
                //startingPose = new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)); //TODO
                return new BouncePathAuto(s_Swerve);

            case COUNT321_AUTO:
                //startingPose = new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0));
                return new  Count321Path(s_Swerve);

            case SWERVE1126:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new Swerve1126(s_Swerve);
            case CURVE_PATH:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new CurvePath(s_Swerve);
                
            case PATH_PLANNER_STRAIGHT:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new PathPlannerStraight(s_Swerve);
            case CURVE_LINE_WAYPOINT:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new CurveLineWayPoint(s_Swerve);

            case PATH_PLANNER_BOUNCE:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new PathPlannerBounce(s_Swerve);
            case STRAIGHT_LINE_MODE:
                //startingPose = new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0));
                return new StraightLineMode(s_Swerve);
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
