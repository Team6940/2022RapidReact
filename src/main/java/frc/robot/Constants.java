// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    // Turret Constants
    public static final int turretID = 22; //TODO
    public static final int kTurretMaxSpeed = 400; //TODO
    public static final int kTurretStartingAngle = 0; //TODO
    public static final int kTurretAngleTolerance = 0; //TODO
    public static final boolean kOutputTelemetry = true;

    //LED Constants
    public static final int LED_PORT = 5;  /*LEDs PWM port */  //TODO
    public static final int LED_LENGTH = 60;  /*LEDs pixels length */ //TODO

    // Limelight Constants
  
    public static final double kHorizAngleCorrection = 2.5;   // + is left
    public static final double LL_MOUNT_HEIGHT = 1.0;  /* limelight 固定height */  //TODO
    public static final double LL_MOUNT_ANGLE = 30; /* limelight固定角度 */   //TODO
    // Shooter Constants
    public static final double SHOOTER_LAUNCH_ANGLE = 70.0; //SHOOTER固定角度  //TODO
    public static final double SHOOTER_MOUNT_HEIGHT = 1.1;  //SHOOTER高度  //TODO
    public static final int SHOOT_L_MASTER_ID = 20;  //TODO
    public static final int SHOOT_R_MASTER_ID = 21;  //TODO

    public static final double kFlywheelIdleVelocity = 1.0; //meters/s //TODO
    public static double kFlyWheelEncoderReductionRatio =  1.0 / 2.0 ;  //TODO
    public static double kFlyWheelWheelDiameter = 0.093;//The unit is meter //TODO
    public static double kFlyWheelWheelDefaultSpeed = 3.0;  //meters/s //TODO
    public static double kShooterTolerance = 0.01;  //meters/s //TODO
    

    
    // Goal Constants
    public static final double LL_UPPER_HUB_HEIGHT = 2.64;
    public static final double CARGO_DIAMETER = 0.64;
    public static final double UPPER_HUB_DIAMETER = 1.22;
    
    public static double kDriveMotorMaxOutput = 1;
    public static double kPivotMotorMaxOutput = 1;

    public static double kDriveMotorNeutralDeadband = 0;
    public static double kPivotMotorNeutralDeadband = 0;

    public static double stickDeadband = 0.15;

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
    public static double kDriveMotorkP = 0.025; // 5e-2 0.05   0.025
    public static double kDriveMotorkI = 0.0016; //5e-4 0.005  0.0016
    public static double kDriveMotorkD = 2.5; //   5e-0 5 1.5  2.5
    public static double kDriveMotorkF = 0.06;//   0.045       0.06
    public static double kDriveMotorIZone = 240;// 90          240

    public static double kPivotMotorkP = 2;//3
    public static double kPivotMotorkI = 0;
    public static double kPivotMotorkD = 100;//100
    public static double kPivotMotorF = 0;
    public static double kPivotMotorkIZone = 0;
    public static int continuousCurrentLimit = 10;
    public static double motionCruiseVelocity = 1200;
    public static double motionAcceleration = 3500;
    public static double velocityMeasurementWindow = 64;
    public static double voltageCompSaturation = 12;
    

    public static double kLoopSeconds = 0.0;

    public static double kDriveMotorReductionRatio = 1.0 / (29/15*60/15); //29/15*60/15
    public static double kPivotMotorReductionRatio = 1.0 / (56/6*60/10); //56/6*60/10
    public static double kDriveEncoderReductionRatio =  1.0 / (29/15*60/15);
    public static double kPivotEncoderReductionRatio = -1.0 / 1.0;

    public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
    public static double kPivotEncoderResolution = 4096.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad

    public static double kWidth  = 0.572936;//The unit is 0.342_m
    public static double kLength = 0.572936;//The unit is 0.342_m

    public static double kWheelDiameter = 0.093;//The unit is meter

    public static double kPeriod = 20;//The unit is 20_ms

    public static double kMaxSpeedinTeleop = 4;
    public static double kMaxSpeed = 4;//The unit is meters per second
    public static double kMaxOmega = 5;//The unit is 12_rad_per_s

    public static final PIDController holonomicControllerPID = new PIDController(1, 0, 0);
    public static final ProfiledPIDController holonomicControllerPIDTheta = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
        new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));

    public static final double wheelCircumference = kWheelDiameter * Math.PI;

    public static final double driveGearRatio = 29.0/15*60/15;
    public static final double angleGearRatio = 56.0/6*60/10; 

    public static final double MAX_SPEED_METERSperSECOND = 21900/0.1/(driveGearRatio*2048)*wheelCircumference;
    public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*(kWidth/2*1.414213);//Math.sqrt(2)
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/METERSperROBOT_REVOLUTION*(2*Math.PI);

    public static final class SwerveConstants {	
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = 29/15*60/15;
        public static final double angleGearRatio = 56/6*60/10; 

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.58526 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.8652 / 12);
        public static final double driveKA = (0.17455 / 12);
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
				3*Math.PI, 3*Math.PI);
				
		// Trajectory Speed Configs
		public static final TrajectoryConfig defaultConfig = 
		new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics);

		public static final TrajectoryConfig slowConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics);
		
		public static final TrajectoryConfig fastConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics);
			
		public static final TrajectoryConfig RTNfastConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics);

		public static final TrajectoryConfig RTNFastToZero = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics)
			.setStartVelocity(Constants.AutoConstants.kFastMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
		
		public static final TrajectoryConfig zeroToSlow =
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics)
			.setStartVelocity(0)
			.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);

		public static final TrajectoryConfig slowToZero =
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
            .setKinematics(swerveKinematics)
    		.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
			
		public static final TrajectoryConfig zeroToMax =
        	new TrajectoryConfig(
            	Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics)
			.setStartVelocity(0.0)
			.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
			
		    
		public static final TrajectoryConfig maxToZero =
			new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(swerveKinematics)   
			.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
      }


}
