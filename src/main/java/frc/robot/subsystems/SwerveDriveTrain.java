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
    PixySignature = SmartDashboard.getBoolean("Debug/Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

    SmartDashboard.putData("Debug/Drive/Field", m_field);

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
  }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return odometry_.getPoseMeters();
    }


  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(
            getStates());
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

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return new ChassisSpeeds(
        getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos() - getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
        getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos() + getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  public double getFieldRelativeXVelocity() {
      return getFieldRelativeChassisSpeeds().vxMetersPerSecond;
  }

  public double getFieldRelativeYVelocity() {
      return getFieldRelativeChassisSpeeds().vyMetersPerSecond;
  }

  public double getFieldRelativeAngularVelocity() {
      return getFieldRelativeChassisSpeeds().omegaRadiansPerSecond;
  }

  public double getRadialVelocity(){
    return getFieldRelativeXVelocity() * Math.cos(getFieldRelativeTurretAngleRad()) + getFieldRelativeYVelocity() * Math.sin(getFieldRelativeTurretAngleRad());
  }

  public double getTangentialVelocity(){
    return getFieldRelativeXVelocity() * Math.sin(getFieldRelativeTurretAngleRad()) - getFieldRelativeYVelocity() * Math.cos(getFieldRelativeTurretAngleRad());
  }

  public double getFieldRelativeTurretAngleDeg(){
    return Turret.getInstance().getAngleDeg() + LimelightSubsystem.getInstance().Get_tx() + GetHeading_Deg();
  }

  public double getFieldRelativeTurretAngleRad(){
    return Math.toRadians(getFieldRelativeTurretAngleDeg());
  }

  public double getFieldRelativeReadyTurretAngleDeg(){
    return Turret.getInstance().getAngleDeg() + GetHeading_Deg();
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

  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = getStates();
    // This method will be called once per scheduler run
    odometry_.update(
      GetGyroRotation2d(), 
      moduleStates);

    m_field.setRobotPose(getPose());

    SmartDashboard.putNumber("Debug/Drive/x meters", getPose().getX());
    SmartDashboard.putNumber("Debug/Drive/y meters", getPose().getY());
    SmartDashboard.putNumber("Debug/Drive/rot radians", getPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Debug/Drive/isOpenloop", isOpenLoop);
  }
}
