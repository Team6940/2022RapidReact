// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveControl.SwerveControll;
import frc.robot.auto.AutonomousSelector;
import frc.robot.commands.Limelight.AutoAim;
import frc.robot.commands.Limelight.RotateDrivetrainByLimelightAngle;
import frc.robot.subsystems.AimManager;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberNew;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.FeedManager;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotTracker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveTrain;
//import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.FeedManager.FeedManagerState;
import frc.robot.subsystems.Hopper.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance ;
  // The robot's subsystems and commands are defined here...
  public static XboxController m_driverController = new XboxController(0);
  public static XboxController m_operatorController = new XboxController(1);

  public static SwerveDriveTrain m_swerve;

  //public static LedSubsystem m_leds;
  public static LimelightSubsystem m_limelight;
  //public static VisionManager m_visionManager;
  public static AimManager m_aimManager;
  public static Turret m_turret;
  //public static Shooter m_shooter;
  public static Shooter m_shooter;
  public static Climber m_climber;
  public static ClimberNew m_climberNew;
  public static RobotTracker m_robotTracker;
  public static Hopper m_hopper;
  public static Intake m_intake;
  public static ColorSensor m_colorsensor;
  public static FeedManager m_feedmanager;
  public static int autoShootMode = 1;
  private final AutonomousSelector autonomousSelector;

  // Swerve Driver's buttons
  public static JoystickButton limelightButton;
  public static JoystickButton resetyawButton;
  public static JoystickButton resetOdometryButton;
  public static JoystickButton manualLimelightButton;
  public double triggerLeft;
  public double triggerRght;

  // Operator's buttons
  public static JoystickButton IntakeButton;
  public static JoystickButton BlockerButton;
  public static JoystickButton ClimberButton;
  public static JoystickButton ShooterSwitchModeButton;
  //public static JoystickButton DontShootButton;
  public static JoystickButton feederButton;
  public static JoystickButton ShootParaButton;
  public static JoystickButton testTopBallButton;
  public static JoystickButton testBottomBallButton;
  public static JoystickButton testWrongBallButton;

  public static RobotContainer getInstance() {
    if (instance == null){
        instance = new RobotContainer();
    }
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_swerve = SwerveDriveTrain.getInstance();

    m_limelight = LimelightSubsystem.getInstance();
    //m_leds = LedSubsystem.getInstance();
    //m_leds.conformToState(LedSubsystem.State.INVISIBLE_TARGET_TRACKING);
    //m_visionManager = VisionManager.getInstance();
    m_aimManager = AimManager.getInstance();
    m_turret = Turret.getInstance();  
    m_shooter = Shooter.getInstance();
    //m_climber = Climber.getInstance();
    m_climberNew = ClimberNew.getInstance();
    m_robotTracker = RobotTracker.getInstance();
    m_colorsensor = ColorSensor.getInstance();
    m_hopper = Hopper.getInstance();
    m_intake = Intake.getInstance();
    m_feedmanager = FeedManager.getInstance();

    // The Swerve Driver's buttons
    limelightButton = new JoystickButton(m_driverController, 6);
    resetyawButton = new JoystickButton(m_driverController, 2);
    resetOdometryButton = new JoystickButton(m_driverController, 3);
    IntakeButton = new JoystickButton(m_driverController, 5);
    triggerLeft = m_driverController.getLeftTriggerAxis();
    triggerRght = m_driverController.getRightTriggerAxis();

    // The operator's buttons
    BlockerButton = new JoystickButton(m_operatorController, 6);
    ClimberButton = new JoystickButton(m_operatorController, 3);
    //ShooterSwitchModeButton = new JoystickButton(m_operatorController, 9);
    //DontShootButton = new JoystickButton(m_operatorController, 7);
    feederButton = new JoystickButton(m_operatorController, 7);
    ShootParaButton = new JoystickButton(m_operatorController, 4);
    if (RobotBase.isSimulation()){    
      testBottomBallButton = new JoystickButton(m_operatorController, 5);
      testTopBallButton = new JoystickButton(m_operatorController, 9);
      testWrongBallButton  = new JoystickButton(m_operatorController, 10);
    }

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
    //limelightButton.whenHeld(new AutoAim());
    limelightButton.whileActiveContinuous(new RotateDrivetrainByLimelightAngle(true));

    // Hopper button
    //IntakeButton.whenHeld(new InstantCommand(() ->m_intake.runIntaker()));
    IntakeButton.whenPressed(new InstantCommand(() -> m_intake.stopSolenoid()));

    // Blocker button
    BlockerButton.whenHeld(new InstantCommand(() -> m_shooter.setFiring(true)));
    BlockerButton.whenReleased(new InstantCommand(() -> m_shooter.setFiring(false)));

    // Climber button
    ClimberButton.whenPressed(new InstantCommand(() -> m_climberNew.autoturnclimber()));

    // Reset Yaw button . Remember to protect it during the game!
    resetyawButton.whenPressed(new InstantCommand(() -> m_swerve.ZeroHeading()));

    /* Use the method below if the head PID Control is wanted*/
    //resetyawButton.whenReleased(new InstantCommand(() -> m_swerve.WhetherStoreYaw()));

    resetOdometryButton.whenPressed(new InstantCommand(() -> m_swerve.resetOdometry()));

    // ShootPara debug Button 
    ShootParaButton.whenPressed(new InstantCommand(() -> m_aimManager.DebugShootParameter()));

    // simulation test Button
    if (RobotBase.isSimulation()) {
      testBottomBallButton.whenPressed(new InstantCommand(() -> m_hopper.DoBottomBallTest()));
      testTopBallButton.whenPressed(new InstantCommand(() -> m_hopper.DoTopBallTest()));
      testWrongBallButton.whenPressed(new InstantCommand(() -> m_colorsensor.DoWrongBallTest()));
    }

    feederButton.whenPressed(new InstantCommand(() -> m_feedmanager.switchFeederMode()));
    /**
     * Below are the buttons when we use a turret
     */

    // Shooter button
    //ShooterSwitchModeButton.whenPressed(new InstantCommand(() -> m_visionManager.autoSwitchShooterMode()));

    // Turret button
    /*
    DontShootButton.whenHeld(
      new SequentialCommandGroup(
        new InstantCommand(() -> m_limelight.setLightMode(1)),
        new InstantCommand(() -> m_visionManager.Stop())
        )
      );
    DontShootButton.whenReleased(
      new SequentialCommandGroup(
        new InstantCommand(() -> m_limelight.setLightMode(3)),
        new InstantCommand(() -> m_visionManager.startVisionFinding())
        )
      );*/
  }
  
  public double getSpeedScaledDriverLeftX() {
    return -m_driverController.getLeftX();
  }

  public double getSpeedScaledDriverLeftY() {
    return -m_driverController.getLeftY();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return autonomousSelector.getCommand(m_swerve);
  }

  public void runTeleInitCommand() {
    if (getAutonomousCommand() != null) {
      getAutonomousCommand().cancel();
    }
  }
}
