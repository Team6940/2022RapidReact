// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveControl.SwerveControll;
import frc.robot.auto.AutonomousSelector;
import frc.robot.commands.Limelight.AutoAim;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberOld;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController m_driverController = new XboxController(0);
  public static XboxController m_operatorController = new XboxController(1);

  public static SwerveDriveTrain m_swerve;

  public static LedSubsystem m_leds;
  public static LimelightSubsystem m_limelight;
  public static Turret m_turret;
  public static Shooter m_shooter;
  public static Feeder m_feeder;
  public static ClimberOld m_climber;
  public static Blocker m_blocker;
  public static Conveyor m_conveyor;

  private final AutonomousSelector autonomousSelector;

  // Swerve Driver's buttons
  public static JoystickButton limelightButton;
  public static JoystickButton resetyawButton;
  public static JoystickButton controlopenlooptypeButton;
  public static JoystickButton controlclosedlooptypeButton;
  public static JoystickButton IntakeButton;
  public static JoystickButton resetOdometryButton;
  public static JoystickButton PixyButton;

  // Operator's buttons
  public static JoystickButton BallLoadButton;
  public static JoystickButton BlockerButton;
  public static JoystickButton ElasticClimberButton;
  public static JoystickButton ElasticClimberStopButton;
  public static JoystickButton StraightClimberButton;
  public static JoystickButton ShooterSwitchModeButton;
  public static JoystickButton TurretButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_swerve = SwerveDriveTrain.getInstance();

    m_limelight = LimelightSubsystem.getInstance();
    m_leds = LedSubsystem.getInstance();
    m_leds.conformToState(LedSubsystem.State.INVISIBLE_TARGET_TRACKING);
    m_turret = Turret.getInstance();
    m_shooter = Shooter.getInstance();
    m_feeder = Feeder.getInstance();
    m_climber = ClimberOld.getInstance();
    m_blocker = Blocker.getInstance();
    m_conveyor = Conveyor.getInstance();

    // The Swerve Driver's buttons
    limelightButton = new JoystickButton(m_driverController, 6);
    resetyawButton = new JoystickButton(m_driverController, 3);
    resetOdometryButton = new JoystickButton(m_driverController, 1);
    IntakeButton = new JoystickButton(m_driverController, 7);
    controlopenlooptypeButton  = new JoystickButton(m_driverController, 4);
    controlclosedlooptypeButton = new JoystickButton(m_driverController, 5);
    PixyButton = new JoystickButton(m_driverController, 8);

    // The operator's buttons
    BallLoadButton = new JoystickButton(m_operatorController, 1);
    BlockerButton = new JoystickButton(m_operatorController, 2);
    ElasticClimberButton = new JoystickButton(m_operatorController, 3);
    ElasticClimberStopButton = new JoystickButton(m_operatorController, 4);
    StraightClimberButton = new JoystickButton(m_operatorController, 5);
    ShooterSwitchModeButton = new JoystickButton(m_operatorController, 6);
    TurretButton = new JoystickButton(m_operatorController, 7);

    m_swerve.setDefaultCommand(new SwerveControll());

    autonomousSelector = new AutonomousSelector();

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
    // Limelight button
    limelightButton.whenHeld(new AutoAim());

    // Intake button
    IntakeButton.whenPressed(new InstantCommand(() -> m_feeder.autoturnintaker()));
  
    // Ball Lodaer button
    BallLoadButton.whenHeld(new InstantCommand(() -> m_feeder.turnonballLoader()));
    BallLoadButton.whenReleased(new InstantCommand(() -> m_feeder.turnoffballLoader()));

    // Blocker button
    BlockerButton.whenHeld(new InstantCommand(() -> m_blocker.turnonballLocker()));
    BlockerButton.whenReleased(new InstantCommand(() -> m_blocker.turnoffballLocker()));

    // Climber button
    //ElasticClimberButton.whenPressed(new InstantCommand(() -> m_climber.autosetElasticClimber()));
    //ElasticClimberStopButton.whenPressed(new InstantCommand(() -> m_climber.stopElasticClimber()));
    //StraightClimberButton.whenPressed(new InstantCommand(() -> m_climber.autosetStraighClimber()));
    ElasticClimberButton.whenPressed(new InstantCommand(() -> m_climber.autoStartClimb()));
    ElasticClimberStopButton.whenPressed(new InstantCommand(() -> m_climber.stopClimb()));

    // Shooter button
    ShooterSwitchModeButton.whenPressed(new InstantCommand(() -> m_shooter.autoSwitchShooterMode()));

    // Turret button
    TurretButton.whenHeld(
      new SequentialCommandGroup(
        new InstantCommand(() -> m_limelight.setLightMode(3)),
        new InstantCommand(() -> m_turret.startVisionFinding()),
        new InstantCommand(() -> m_shooter.setInitShooter()) )
      );
    TurretButton.whenReleased(
      new SequentialCommandGroup(
        new InstantCommand(() -> m_limelight.setLightMode(1)),
        new InstantCommand(() -> m_turret.Stop()),
        new InstantCommand(() -> m_shooter.setStopShooter()) )
      );

    // Reset Yaw button . Remember to protect it during the game!
    resetyawButton.whenPressed(new InstantCommand(() -> m_swerve.ZeroHeading()));
    resetyawButton.whenReleased(new InstantCommand(() -> m_swerve.WhetherStoreYaw()));

    PixyButton.whenHeld(new InstantCommand(() -> m_swerve.turnOnPixy()));
    PixyButton.whenReleased(new InstantCommand(() -> m_swerve.turnOffPixy()));

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
      return autonomousSelector.getCommand(m_swerve);
  }
}
