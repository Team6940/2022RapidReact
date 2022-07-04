// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.pseudoresonance.pixy2api.*;

public class SwerveDriveTrain extends SubsystemBase {

  private static SwerveDriveTrain instance = null;
  /** Creates a new SwerveDriveTrain. */
  private SwerveModule swerve_modules_[] = new SwerveModule[4];

  public PigeonIMU gyro;
  public AHRS ahrs;
  public PixyCamSPI mPixy;
  byte PixySignature;

  public boolean auto = false;

  double preverror;
  double responsetime = 0.02;
  int i = 1;
  double HEAD_P = 0.01;
  double HEAD_I = 0;
  double HEAD_D = 0;
  PIDController headController = new PIDController(HEAD_P, HEAD_I, HEAD_D);

  double vxMetersPerSecond;
  double vyMetersPerSecond;

  public boolean isOpenLoop = true;

  public boolean whetherstoreyaw = false;

  public boolean autoPixy = false;

  private Field2d m_field = new Field2d();

  public final static SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d( Constants.kLength / 2,  Constants.kWidth / 2),//front left
        new Translation2d( Constants.kLength / 2, -Constants.kWidth / 2),//front right
        new Translation2d(-Constants.kLength / 2,  Constants.kWidth / 2),//back left
        new Translation2d(-Constants.kLength / 2, -Constants.kWidth / 2)//back right
  );

  public SwerveDriveOdometry odometry_ =
      new SwerveDriveOdometry(
        Constants.swerveKinematics,
        new Rotation2d(0),
        new Pose2d()
  );

  public SwerveDriveTrain() {
    gyro = new PigeonIMU(Constants.PigeonIMUPort);

    // The coordinate system may be wrong 
    swerve_modules_[0] = new SwerveModule(1, 2, true,  false, 1814, false, false);//front left
    swerve_modules_[1] = new SwerveModule(3, 4, true, false, 3570, true,  true);//front right
    swerve_modules_[2] = new SwerveModule(5, 6, true,  false, -370,  true, true);//back left
    swerve_modules_[3] = new SwerveModule(7, 8, true, false, 2302,  true, true);//back right
    
    ahrs = new AHRS(SPI.Port.kMXP);

    mPixy = PixyCamSPI.getInstance();
    /* select cargo color for sig */
    PixySignature = SmartDashboard.getBoolean("Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

    SmartDashboard.putData("Field", m_field);

  }
  
  public static SwerveDriveTrain getInstance() {
    if (instance == null){
      instance = new SwerveDriveTrain();
    }
    return instance;
  }

  public void Drive(Translation2d translation,double omega,boolean fieldRelative,boolean isOpenloop){
    var states = Constants.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(), translation.getY(), omega, GetGyroRotation2d())
      : new ChassisSpeeds(translation.getX() , translation.getY(), omega)
    );

    vxMetersPerSecond =       
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(), 
        omega, 
        GetGyroRotation2d()).vxMetersPerSecond;

    vyMetersPerSecond =       
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(), 
        omega, 
        GetGyroRotation2d()).vyMetersPerSecond;

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kMaxSpeed);
      
    for(int i = 0;i < swerve_modules_.length;i++ ){
      swerve_modules_[i].SetDesiredState(states[i],isOpenloop);
    }
  }

  public void SetModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeed);
      for(int i = 0;i < swerve_modules_.length;i++){
        swerve_modules_[i].SetDesiredState(desiredStates[i], true);
      }
      // 
      var frontLeftState = desiredStates[0];
      var frontRightState = desiredStates[1];
      var backLeftState = desiredStates[2];
      var backRightState = desiredStates[3];

      // Convert to chassis speeds
      ChassisSpeeds chassisSpeeds = Constants.swerveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);

      //Get field-relative x and y speed
      //TODO:The gyro may needs to change
      vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond * GetGyroRotation2d().getCos() + chassisSpeeds.vyMetersPerSecond * GetGyroRotation2d().getSin();
      vyMetersPerSecond = - chassisSpeeds.vxMetersPerSecond * GetGyroRotation2d().getSin() + chassisSpeeds.vyMetersPerSecond * GetGyroRotation2d().getCos();
      
  }

  /**
    * Calculate and set the requred SwerveModuleStates for a given ChassisSpeeds
    * 
    * @param speeds
    */
  public void setChassisSpeeds (ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeed);
    SetModuleStates(moduleStates);
  }

  public double GetHeading_Rad(){
    /*The unit is radian */
    return GetGyroRotation2d().getRadians();
  }

  public double GetHeading_Deg(){
    /*The unit is radian */
    return GetGyroRotation2d().getDegrees();
  }

  public void ZeroHeading(){
    whetherstoreyaw = true;
    ahrs.reset();
    //ResetOdometry(new Pose2d());
  }

  public void WhetherStoreYaw(){
    whetherstoreyaw = false;
  }

  public void turnOnPixy(){
    autoPixy = true;
  }

  public void turnOffPixy(){
    autoPixy = false;
  }

  public Pose2d GetPose(){
    return odometry_.getPoseMeters();
  }

  public void ResetOdometry(Pose2d pose){
    odometry_.resetPosition(pose, GetGyroRotation2d());
    //for (int i = 0 ; i < swerve_modules_.length; i++){
    //  swerve_modules_[i].setPose(pose);
    //}
  }

  public double getHeading(){
    return Math.IEEEremainder(ahrs.getAngle(), 360);
  }

  public Rotation2d GetGyroRotation2d(){
    // An offset will be needed if the robot doesn't face downfield
    return ahrs.getRotation2d();
  }

  public double GetYaw(){
    return ahrs.getYaw();
  }

  public double deadband(double x){
    return ((x < 0 ? -x : x) < 0.05) ? 0 : x;
  }

  public double calcYawStraight(double targetAngle, double currentAngle){
    //The WPILIB's version
    double headadjust = headController.calculate(currentAngle, targetAngle);
    return headadjust;
  }

  public double remainderf(double m , double n){
    return (m / n) > 0.0 ? m - Math.floor(m / n) * n : m - Math.ceil(m / n) * n;
  }

  public void setControlModeOpen(){
    isOpenLoop = false;
  }

  public void setControlModeClosed(){
    isOpenLoop = true;
  }

  public void resetOdometry(){
    odometry_.resetPosition(new Pose2d(), new Rotation2d());
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[swerve_modules_.length];
    for (int i = 0;i < swerve_modules_.length;i++ ){
      states[i] = swerve_modules_[i].GetState();
    }
    return states;
  }

  public void zeroGyro(){
    gyro.setFusedHeading(0);
  }

  public void zeroGyro(double reset){
    gyro.setFusedHeading(reset);
  }

  public Rotation2d getYaw() {
      return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public double GetVxSpeed(){
    return vxMetersPerSecond;
  }

  public double GetVySpeed(){
    return vyMetersPerSecond;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = getStates();
    // This method will be called once per scheduler run
    odometry_.update(
      GetGyroRotation2d(), 
      moduleStates);

    m_field.setRobotPose(odometry_.getPoseMeters());

    SmartDashboard.putNumber("x meters", odometry_.getPoseMeters().getX());
    SmartDashboard.putNumber("y meters", odometry_.getPoseMeters().getY());
    SmartDashboard.putNumber("rot radians", odometry_.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putBoolean("isOpenloop", isOpenLoop);
  }
}
