// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveControl.SwerveControll;
import frc.robot.commands.Autos.PathPlanner.PathPlannerBounce;
import frc.robot.commands.Autos.PathPlanner.PathPlannerStraight;
import frc.robot.commands.Autos.PathPlanner.PathPlannerWithTwo;
import frc.robot.commands.Autos.PathWeaver.BouncePathAuto;
import frc.robot.commands.Autos.PathWeaver.Count321Path;
import frc.robot.commands.Autos.PathWeaver.CurveLineWayPoint;
import frc.robot.commands.Autos.PathWeaver.CurvePath;
import frc.robot.commands.Autos.PathWeaver.ExamplePath;
import frc.robot.commands.Autos.PathWeaver.Swerve1126;
import frc.robot.commands.Autos.WayPointsControl.StraightLineMode;
import frc.robot.commands.Limelight.AutoAim;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos.AutonomousSelector;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController m_driverController = new XboxController(0);

  public static SwerveDriveTrain m_swerve;

  public static LedSubsystem m_leds;
  public static Limelight m_limelight;
  public static Turret m_turret;
  public static Shooter m_shooter;


  // limelight button
  public static JoystickButton limelightButton;
  public static JoystickButton resetyawButton;
  public static JoystickButton controlopenlooptypeButton;
  public static JoystickButton controlclosedlooptypeButton;
  private final AutonomousSelector autonomousSelector;
  public static JoystickButton resetOdometryButton;
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  private final BouncePathAuto bounceAuto; 
  private final ExamplePath exampleAuto;
  private final Count321Path count321Auto;
  private final Swerve1126 swerve1126;
  private final StraightLineMode StraightLinedMode;
  private final PathPlannerStraight  pathPlannerStraight;
  private final PathPlannerBounce pathPlannerBounce;
  private final CurvePath curvePath;
  private final CurveLineWayPoint curveLineWayPoint;
  private final PathPlannerWithTwo pathplannerwithtwo;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_swerve = SwerveDriveTrain.getInstance();

    m_limelight = Limelight.getInstance();
    boolean runNewFeature = false;
    if(runNewFeature){
      m_leds = LedSubsystem.getInstance();
      m_leds.conformToState(LedSubsystem.State.INVISIBLE_TARGET_TRACKING);
      m_turret = Turret.getInstance();
      m_shooter = Shooter.getInstance();
      double speed = m_shooter.RpmToMeterSpeed(3000);
      double rpm = m_shooter.meterSpeedToRpm(speed);
    }
    m_limelight.getShooterLaunchVelocity(Constants.SHOOTER_LAUNCH_ANGLE);

    limelightButton = new JoystickButton(m_driverController, 6);
    resetyawButton = new JoystickButton(m_driverController, 3);
    resetOdometryButton = new JoystickButton(m_driverController, 1);
    controlopenlooptypeButton  = new JoystickButton(m_driverController, 4);
    controlclosedlooptypeButton = new JoystickButton(m_driverController, 5);

    m_swerve.setDefaultCommand(new SwerveControll());


    bounceAuto = new BouncePathAuto(m_swerve);
    exampleAuto = new ExamplePath(m_swerve);
    count321Auto = new Count321Path(m_swerve);
    swerve1126 = new Swerve1126(m_swerve);
    StraightLinedMode = new StraightLineMode(m_swerve);
    pathPlannerStraight = new PathPlannerStraight(m_swerve);
    pathPlannerBounce = new PathPlannerBounce(m_swerve);
    curvePath = new CurvePath(m_swerve);
    curveLineWayPoint = new CurveLineWayPoint(m_swerve);
    pathplannerwithtwo = new PathPlannerWithTwo(m_swerve);
 

    autonomousSelector = new AutonomousSelector();

    // Add autos to SmartDashboard
    autoChooser.setDefaultOption("PathPlannerWithTwo", pathplannerwithtwo);
    autoChooser.addOption("Bounce Path", bounceAuto);
    autoChooser.addOption("Straight Line", exampleAuto);
    autoChooser.addOption("Count321Path", count321Auto);
    autoChooser.addOption("Swerve1126", swerve1126);
    autoChooser.addOption("CurvePath", curvePath);
    autoChooser.addOption("PathPlannerStraight", pathPlannerStraight);
    autoChooser.addOption("CurveLineWayPoint", curveLineWayPoint);
    autoChooser.addOption("PathPlannerBounce", pathPlannerBounce);
    autoChooser.addOption("StraightLineMode", StraightLinedMode);

    
    SmartDashboard.putData("Auto", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Limelight Button
    limelightButton.whenHeld(new AutoAim());

    //Reset Yaw Button . Remember to protect it during the game!
    resetyawButton.whenPressed(new InstantCommand(() -> m_swerve.ZeroHeading()));
    resetyawButton.whenReleased(new InstantCommand(() -> m_swerve.WhetherStoreYaw()));

    resetOdometryButton.whenPressed(new InstantCommand(() -> m_swerve.resetOdometry()));

    controlopenlooptypeButton.whenPressed(new InstantCommand(() -> m_swerve.setControlModeOpen()));
    controlclosedlooptypeButton.whenPressed(new InstantCommand(() -> m_swerve.setControlModeClosed()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    boolean newAuto = true;
    if( !newAuto){
      return autoChooser.getSelected();
    }
    else{
      return autonomousSelector.getCommand(m_swerve);
    }
  }
}
