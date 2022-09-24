// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib.team1706.LinearInterpolationTable;

import java.awt.geom.Point2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ShooterConstants {
        public static final double kAccelCompFactor = 0.100;

        public static final double kHangarRPM = 1200;
        
        private static final Point2D[] kHoodPoints = new Point2D.Double[] {
            // (distance, ty-angle)
            new Point2D.Double(2.386/*90*/, 0), //
            new Point2D.Double(2.887/*105*/, 7.00), //
            new Point2D.Double(3.349/*120*/, 12.00), //
            new Point2D.Double(3.914/*135*/, 15.00), //
            //new Point2D.Double(3.810/*150*/, 29.0), //
            //new Point2D.Double(4.191/*165*/, 30.5), //
            //new Point2D.Double(4.572/*180*/, 32.0), //
            //new Point2D.Double(4.953/*195*/, 36.0), //
            //new Point2D.Double(5.334/*210*/, 37.0), //
            //new Point2D.Double(5.715/*225*/, 38.0)//
        };
        public static final LinearInterpolationTable kHoodTable = new LinearInterpolationTable(kHoodPoints);
    
        private static final Point2D[] kRPMPoints = new Point2D.Double[] {
            // (distance, shooterSpeedRPM)
            new Point2D.Double(2.396/*90*/, 2305), //
            new Point2D.Double(2.887/*135*/, 2360), //
            new Point2D.Double(3.349/*150*/, 2405), //
            new Point2D.Double(3.914/*165*/, 2460), //
            //new Point2D.Double(4.572/*180*/, 3040), //
            //new Point2D.Double(4.953/*195*/, 3185), //
            //new Point2D.Double(5.334/*210*/, 3315), //
            //new Point2D.Double(5.715/*225*/, 3500), //
            //new Point2D.Double(6.096/*240*/, 3700),
            //new Point2D.Double(6.858/*270*/, 4000),
        };        
        public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

        private static final Point2D[] kShotTimes = new Point2D.Double[] {
        // (ty-angle,time)
        new Point2D.Double(3.17/*135*/, 7.00),
        };

        public static final LinearInterpolationTable kTimeTable = new LinearInterpolationTable(kShotTimes);

        public static final int SHOOTER_EJECT_SPEED = 1200;

        public static final double SHOOTER_LAUNCH_ANGLE = 90-Math.toDegrees(0.35); //SHOOTER固定角度  //TODO
        public static final double SHOOTER_MOUNT_HEIGHT = 0.83;  //SHOOTER高度  //TODO
        public static final int SHOOT_L_MASTER_ID = 11;  //TODO
        public static final int SHOOT_R_MASTER_ID = 12;  //TODO
        public static final double SHOOTER_KS = 0 / 12;
        public static final double SHOOTER_KV = 0 / 12;
        public static final double SHOOTER_KA = 0 / 12;
    
        public static final double kFlywheelIdleVelocity = 1000; //RPM //TODO
        public static double kFlyWheelEncoderReductionRatio =  1 ;  //TODO
        public static double kFlyWheelWheelDiameter = 0.108;//The unit is meter //TODO
        public static double kFlyWheelWheelDefaultSpeed = 3.0;  //meters/s //TODO
        public static double kShooterTolerance = 20; //RPM //TODO
        public static double kFlyWheelCircumference = Math.PI * kFlyWheelWheelDiameter;
        public static double kShootOneBallTime = 0.2; //TODO every one shooting ball time(seconds)
        public static double kShootTestTime = 60;
        public static double kShootOneWrongBallTime = 0.5 ; //TODO every one shooting ball time(seconds)
    
        public static final double [] angleCoefficients = {-0.00074772,	0.00107806,	-0.00056204, -0.000010622,
                        0.01432893, -0.13352268, 0.00632465, 0.1574279, -0.01956647, 1.49045868};
    
        public static final double [] speedCoefficients = {0.00147817, -0.006165024, 0.006012544, 0.000764929,
                        0.000124517, 0.662020587, -0.003419489, -0.59030299, 0.032468927, 6.108893599};
    }
    
    public static final class GlobalConstants {
        public static final double kLoopTime = 0.020;
        public static final float INF = (float)Math.pow(10, 5); // This represents the Infinite
    }

    public static final class GoalConstants {
        public static final Translation2d kGoalLocation = new Translation2d(8.23, 4.115);
        public static final Translation2d kWrongBallGoal = new Translation2d(5.50, 4.115);
        public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);

        public static final double LL_UPPER_HUB_HEIGHT = 2.64;
        public static final double CARGO_DIAMETER = 0.64;
        public static final double UPPER_HUB_DIAMETER = 1.22;
    }

    public static final class HoodConstants {
        public static final double kMinAngle = 0.5;
        public static final double kMaxAngle = 38;
        public static final double kHoodTolerance = 1.0;//degrees
        
        public static int HoodMotorPort = 14; //14
        public static double HOOD_GEAR_RATIO = 12.7 ;  
        
        public static final double HOOD_HOME_ANGLE = 0; 
        public static final double HOOD_MAX_ANGLE = 20; 
        public static final double HOOD_MIN_ANGLE = 0;
        
        public static final double HOOD_EJECT_ANGLE = 10;  //TODO
    }

    public static final class DriveConstants {
        public static final double kMaxAcceleration = 3.0;
        public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly

        public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                          // read when not in use (Eliminates "Stick Drift")
        public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                          // when aimed at a 45deg angle (Such that X and Y are are
                                                          // maximized simultaneously)
        public static final double kTranslationSlew = 1.45;
        public static final double kRotationSlew = 3.00;
    }
    
    public static final class BlockerConstants {
        public static final int kBlockerID = 9;//TODO
        public static final double kFireSpeed = 0.8;
        public static final double kStopSpeed = 0;
    }

    public static final class HopperConstants {
        public static int HopperPort = 10; //TODO

        public static int HOPPER_TOP_BALL_IR_SENSOR = 1;
        public static int HOPPER_LOW_BALL_IR_SENSOR = 0;

        public static double SLOW_MUL = 5.0 / 8.0;
        public static double TOP_BELT_SPEED = 0.8;
        public static double ACCEPT_SPEED = 1.0;
        public static double REJECT_SPEED = -1.0;

        public static final double HOPPER_SPEED = 1;
        public static final double HOPPER_SLOW_SPEED = HOPPER_SPEED / 2;
    }
    
    public static final class ClimberConstants {
        public static int leftClimberMotorPort = 15;
        public static int rghtClimberMotorPort = 16;
        public static int ClimberSolenoidPort = 0;
        public static double kClimberEncoderReductionRatio = 1.0 / 12.0;

        public static final int CLIMBER_PERIOD = 50;
        public static final int CLIMBER_MOTOR_ID = 25;
        public static final int CLIMBER_MOTOR_2_ID = 26;

        public static final double CLIMBER_MOTOR_KF = 0.0;
        public static final double CLIMBER_MOTOR_KP = 0.2;
        public static final double CLIMBER_MOTOR_KI = 0.0;
        public static final double CLIMBER_MOTOR_KD = 0.0;
        public static final double CLIMBER_MOTOR_IZONE = 0;
        public static final double CLIMBER_MOTOR_MAX_IACCUMULATOR = 0.1;
        public static final double CLIMBER_MOTOR_MAX_OUTPUT = 1;

        public static final int CLIMBER_CURRENT_LIMIT = 37;

        public static final int ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 0;
        public static final int ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 1;

        public static final int PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 2;
        public static final int PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 3;

        public static final int PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL = 4;
        public static final int PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL = 5;

        public static final int PIVOT_SOLENOID_ID = 3;


        public static final double CLIMBER_ENCODER_TICKS_PER_INCH = 2048 * ((68.0 / 9.0) * (32.0 / 24.0)) / (12 * (3.0 / 8.0));

        public static final double CLIMBER_MOTOR_MAX_ERROR = 0.07 * CLIMBER_ENCODER_TICKS_PER_INCH;

        public static final boolean DO_BACK_HOOK_CLIMB = true;

        /**
         * The height to go to once the drivers request the climber to deploy
         */
        public static final double CLIMBER_DEPLOY_HEIGHT = 23.6 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * If the elevator arm is below this height and going down, the climb will abort
         */

        public static final double MIN_CLIMBER_ELEVATOR_HEIGHT = -0.5 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * If the elevator arm is above this height and going down, the climb will abort
         */

        public static final double MAX_CLIMBER_ELEVATOR_HEIGHT = 27.3 * CLIMBER_ENCODER_TICKS_PER_INCH;


        /**
         * How long it takes for the pivot pneumatic to pivot open (become pivoted) (in seconds)
         */
        public static final double ARM_PIVOT_DURATION = 0.5;

        /**
         * How long it takes for the pivot pneumatic to close (become inline) (in seconds)
         */
        public static final double ARM_UNPIVOT_DURATION = 0.5;

        /**
         * How long it takes for the latch pneumatic on the pivot arm to unlatch (in seconds)
         */
        public static final double PIVOT_ARM_UNLATCH_DURATION = 0.3;

        /**
         * Amount (relative) to move the climber arm up to unlatch the elevator arm.
         */
        public static final double CLIMBER_ELEVATOR_UNLATCH_AMOUNT = 5 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * The max safe height for the elevator arm during the swinging part of the climb
         */
        public static final double CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT = 15 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * The height the elevator arm should be at when the climber is doing the final extension to hit the bar
         */
        public static final double MAX_CLIMBER_EXTENSION = 27.07 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * Length to grab onto mid-bar
         */
        public static final double CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION = 0 * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * Length to grab on with front hooks
         */
        public static final double CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_FRONT_HOOK = (27.2 - 3.5) * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * Length to grab with back hooks
         */
        public static final double CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_BACK_HOOK = (27.2 - 3) * CLIMBER_ENCODER_TICKS_PER_INCH;

        /**
         * Roll angle at which the elevator arm won't contact the next bar when extending past the {@link
         * #CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT} (in degrees)
         */
        public static final double ELEVATOR_ARM_SAFE_ANGLE = 42.5;

        /**
         * Roll angle at which the elevator arm is contacting the next bar when extended (in degrees)
         */
        public static final double ON_NEXT_BAR_ANGLE = 41.0;

        /**
         * How long only one of the sensor switches can be closed for before the climb will pause
         */
        public static final double MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME = 0.1;
    }
    
    public static final class IntakeConstants {
        public static final int IntakerPort = 13;
        public static final int IntakerSolenoidPort = 0;
        public static final boolean vSwitchIntake = false;
        public static final double INTAKE_OPEN_TIME = 0.0;
        public static final double INTAKE_SPEED = 1;
        public static final double INTAKE_EJECTION_SPEED = -0.5;
    }

    public static final class RobotTrackerConstants {
        public static final double DRIVE_VELOCITY_MEASUREMENT_LATENCY = 0.0025;
        public static final int ROBOT_TRACKER_PERIOD = 10;
    }
    
    public static final class TurretConstants {
        public static final int turretID = 9; //TODO
        public static final int kTurretMaxSpeed = 400; //TODO
        public static final int kTurretStartingAngle = 0; //TODO
        public static final int kTurretAngleTolerance = 0; //TODO
        public static final int kTurretStep = 10;
        public static final boolean kOutputTelemetry = false;
        public static final double TURRET_GEAR_RATIO = 140.0 / 10.0;
        public static final double TargetMinError = 5.0; //目标锁定的最小误差  //TODO 
        public static final double TurretMaxSoftLimitAngle = 180; // 度数 //TODO
        public static final double TurretMinSoftLimitAngle = -180; // TODO
        public static final double kTolerance = 2 * 2.0; // allowable angle error in degrees for the PIDSubsystem to
        // report atSetpoint() to true
    }

    public static final class LEDConstants {
        public static final int LED_PORT = 5; /*LEDs PWM port */ //TODO
        public static final int LED_LENGTH = 60; /*LEDs pixels length */ //TODO
    }
    
    public static final class LimelightConstants {
            // Limelight Constants
    public static final double kHorizAngleCorrection = 2.5;   // + is left
    public static final double LL_MOUNT_HEIGHT = 0.875;  /* limelight 固定height */  //TODO
    public static final double LL_MOUNT_ANGLE = 30; /* limelight固定角度 */   //TODO
    public static final double kTrackTolerance = 1.146; // Allowable Limelight angle(degree) error in radians //TODO
    }

    public static final class SwerveConstants {	
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = 29/15*60/15;
        public static final double angleGearRatio = 56/6*60/10; 

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.69552 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.8378 / 12);
        public static final double driveKA = (0.44473 / 12);

        // Pigeon Port
        public static final int PigeonIMUPort = 17;

        public static double kDriveMotorMaxOutput = 1;
        public static double kPivotMotorMaxOutput = 1;
    
        public static double kDriveMotorNeutralDeadband = 0;
        public static double kPivotMotorNeutralDeadband = 0;
    
        /**
         * The first version of PID parameters
         *                         Value
         * @param kDriveMotorkP    0.025
         * @param kDriveMotorkI    0.0016
         * @param kDriveMotorkD    2.5
         * @param kDriveMotorkF    0.06
         * @param kDriveMotorIZone 240
         * 
         * 车子在行驶过程中基本不抖动，底盘PID大部分情况下是正常的。
         * 
         */
        public static double kDriveMotorkP = 0.1; // 5e-2 0.05   0.025
        public static double kDriveMotorkI = 0; //5e-4 0.005  0.0016
        public static double kDriveMotorkD = 0; //   5e-0 5 1.5  2.5
        public static double kDriveMotorkF = 0.042;//   0.045       0.06
        public static double kDriveMotorIZone = 0;// 90          240
        public static double kSensorVelocityMeasPeriod = 10;
    
        public static double kPivotMotorkP = 3;//3
        public static double kPivotMotorkI = 0;
        public static double kPivotMotorkD = 100;//100
        public static double kPivotMotorF = 0;
        public static double kPivotMotorkIZone = 0;
        public static double motionCruiseVelocity = 1200;
        public static double motionAcceleration = 3500;    
    
        public static double kLoopSeconds = 0.0;
    
        public static double kDriveMotorReductionRatio = 1.0 / (29/15*60/15); //29/15*60/15
        public static double kPivotMotorReductionRatio = 1.0 / (56/6*60/10); //56/6*60/10
        public static double kDriveEncoderReductionRatio = 1.0 / (29 / 15 * 60 / 15);
        public static double kDriveEncoderReductionRatioTest = (29 / 15 * 60 / 15);
        public static double kPivotEncoderReductionRatio = -1.0 / 1.0;
    
        public static final double FALCON_TICS_PER_ROTATION = 2048.0;
        public static final double TALON_TICS_PER_ROTATION = 4096.0;
    
        public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
        public static double kPivotEncoderResolution = 4096.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
    
        public static double kWidth  = 0.572936;//The unit is 0.342_m
        public static double kLength = 0.572936;//The unit is 0.342_m
    
        public static double kWheelDiameter = 0.093;//The unit is meter
    
        public static double kPeriod = 20;//The unit is 20_ms
    
        public static double kMaxSpeed = 4;//The unit is meters per second
    
        public static final PIDController holonomicControllerPID = new PIDController(1, 0, 0);
        public static final ProfiledPIDController holonomicControllerPIDTheta = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));
    
        public static final double wheelCircumference = kWheelDiameter * Math.PI;
    
        public static final double MAX_SPEED_METERSperSECOND = 21900/0.1/(driveGearRatio*2048)*wheelCircumference;
        public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*(kWidth/2*1.414213);//Math.sqrt(2)
        public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/METERSperROBOT_REVOLUTION*(2*Math.PI);
    
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 16;
	
		public static final double kSlowMaxSpeedMetersPerSecond = 2.0;
		public static final double kSlowMaxAccelerationMetersPerSecondSquared = 3;

		public static final double kFastMaxSpeedMetersPerSecond = 4;
		public static final double kFastMaxAccelerationMetersPerSecondSquared = 3;
		
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 2;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                        3 * Math.PI, 3 * Math.PI);
                
                        public static final TrapezoidProfile.Constraints kThetaAimControllerConstraints =
                        new TrapezoidProfile.Constraints(
                            Math.PI, Math.PI);
    }
    /* new colorSensor */
    public static final class ColorConstants{
        public static final Color kBlueTarget = new Color(0.215, 0.434, 0.350);
        public static final Color kRedTarget = new Color(0.335, 0.421, 0.242);
        public static final Color kNoTarget = new Color(0.256, 0.457, 0.287);
      }
    
}
