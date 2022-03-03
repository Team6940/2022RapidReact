// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.pixy.PixyCamSPI;
import io.github.pseudoresonance.pixy2api.*;

public class SwerveDriveTrain extends SubsystemBase {

  private static SwerveDriveTrain instance = null;
  /** Creates a new SwerveDriveTrain. */
  private SwerveModule swerve_modules_[] = new SwerveModule[4];

  public AHRS ahrs;
  public PixyCamSPI PixyCamSPI;
  byte PixySignature;

  public boolean auto = false;

  double preverror;
  double responsetime = 0.02;
  int i = 1;
  double HEAD_P = 0.036;
  double HEAD_I = 0;
  double HEAD_D = 0;
  PIDController headController = new PIDController(HEAD_P, HEAD_I, HEAD_D);

  public boolean isOpenLoop = true;

  public boolean whetherstoreyaw = false;

  private double[] fieldCentricSpeeds = { 0, 0 };

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

  private double lastHeading = 0; //The unit is radian

  public SwerveDriveTrain() {
    // The coordinate system may be wrong 
    swerve_modules_[0] = new SwerveModule(1, 2, true,  false, 1814, false, false);//front left
    swerve_modules_[1] = new SwerveModule(3, 4, true, false, 3570, true,  true);//front right
    swerve_modules_[2] = new SwerveModule(5, 6, true,  false, -370,  true, true);//back left
    swerve_modules_[3] = new SwerveModule(7, 8, true, false, 2302,  true, true);//back right
    
    ahrs = new AHRS(SPI.Port.kMXP);
    //ZeroHeading();

    PixyCamSPI = new PixyCamSPI(0);
    /* select cargo color for sig */
    PixySignature = SmartDashboard.getBoolean("Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

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
      
      for (int i = 0;i < swerve_modules_.length;i++ ){
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
     * Calculate and set the requred SwerveModuleStates for a given ChassisSpeeds
     * 
     * @param speeds
     */
    public void setChassisSpeeds (ChassisSpeeds speeds) {
      SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeed);
      SetModuleStates(moduleStates);
  }

  public double GetHeading(){
    /*The unit is radian */
    return GetGyroRotation2d().getRadians();
  }

  public void ZeroHeading(){
    whetherstoreyaw = true;
    ahrs.reset();
    //ResetOdometry(new Pose2d());
  }

  public void WhetherStoreYaw(){
    whetherstoreyaw = false;
  }

  public double GetTurnRate(){
    double ret = (GetHeading() - lastHeading) / Constants.kPeriod;
    lastHeading = GetHeading();
    return ret;
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

  /**
  * Set the odometry readings
  * It seems that it will be used in autonomous mode
  * 
  * @param pose Pose to be written to odometry
  * @param rotation Roatation to be written to odometry
  */
  public void autoresetOdometry (Pose2d pose, Rotation2d rotation) {
    odometry_.resetPosition(pose, rotation);
  }
  public double  getGyroAngelDegree()  {
    return ahrs.getAngle();
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

  public double calcYawStraight(double targetAngle, double currentAngle,double kP, double kD){
    //The WPILIB's version
    double headadjust = headController.calculate(currentAngle, targetAngle);

    double errorAngle = remainderf((targetAngle - currentAngle), (double)360);
    if(i == 1){
      preverror = errorAngle;
    }
    double derative = (errorAngle - preverror) / responsetime;
    double correction = errorAngle * kP + derative * kD;
    preverror = errorAngle;
    i++;

    SmartDashboard.putNumber("origin_correction", correction);
    SmartDashboard.putNumber("headadjust", headadjust);
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

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[swerve_modules_.length];
    for (int i = 0;i < swerve_modules_.length;i++ ){
      states[i] = swerve_modules_[i].GetState();
    }
    return states;
}
  @Override
  public void periodic() {

    SwerveModuleState[] moduleStates = getStates();
    // This method will be called once per scheduler run
      odometry_.update(
      GetGyroRotation2d(), 
      moduleStates);

    SmartDashboard.putNumber("x meters", odometry_.getPoseMeters().getX());
    SmartDashboard.putNumber("y meters", odometry_.getPoseMeters().getY());
    SmartDashboard.putNumber("rot radians", odometry_.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("gyroAngle",GetGyroRotation2d().getDegrees());
    
    //SmartDashboard.putNumber("GetSpeed0", swerve_modules_[0].GetSpeed());
    //SmartDashboard.putNumber("GetSpeed1", swerve_modules_[1].GetSpeed());
    //SmartDashboard.putNumber("GetSpeed2", swerve_modules_[2].GetSpeed());
    //SmartDashboard.putNumber("GetSpeed3", swerve_modules_[3].GetSpeed());

    
    //SmartDashboard.putNumber("fr0 angle", swerve_modules_[0].GetState().angle.getDegrees());
    //SmartDashboard.putNumber("fr1 angle", swerve_modules_[1].GetState().angle.getDegrees());
    SmartDashboard.putNumber("fr0 relunit", swerve_modules_[0].GetAngleRelUnits());
    SmartDashboard.putNumber("fr1 relunit", swerve_modules_[1].GetAngleRelUnits());
    SmartDashboard.putNumber("fr0 absunit", swerve_modules_[0].GetAngleAbsUnits());
    SmartDashboard.putNumber("fr1 absunit", swerve_modules_[1].GetAngleAbsUnits());

    /*SmartDashboard.putNumber("fr2 angle", swerve_modules_[2].GetState().angle.getDegrees());
    SmartDashboard.putNumber("fr3 angle", swerve_modules_[3].GetState().angle.getDegrees());
    SmartDashboard.putNumber("robot angle", GetGyroRotation2d().getDegrees());*/
    SmartDashboard.putBoolean("isOpenloop", isOpenLoop);
    boolean runNewFeature = false;
    if(runNewFeature){
      ChassisSpeeds  chassisSpeeds = Constants.swerveKinematics.toChassisSpeeds(moduleStates);
      // robot calculated X, Y velocity
      double dX = chassisSpeeds.vxMetersPerSecond;
      double dY = chassisSpeeds.vyMetersPerSecond;   
      double velMag = Math.sqrt((dX * dX) + (dY * dY)); // magnitude of the velocity vector
      double velAngle = Math.atan2(dY, dX) + Math.toRadians(-GetGyroRotation2d().getDegrees() % 360); // angle of velocity
      // vector +
      // actual robot angle
      //SmartDashboard.putNumber("Field Centric Robot velAngle", Math.toDegrees(getHeading()));
      // field centric X and Y acceleration
      double fieldCentricDX = velMag * Math.cos(velAngle);
      double fieldCentricDY = velMag * Math.sin(velAngle);

      SmartDashboard.putNumber("Field Centric Robot dX", fieldCentricDX);
      SmartDashboard.putNumber("Field Centric Robot dY", fieldCentricDY);

      setFieldCentricSpeeds(fieldCentricDX, fieldCentricDY);
    }
  }

  public synchronized double[] getFieldCentricSpeeds() {
    return fieldCentricSpeeds;
  }
  public void setFieldCentricSpeeds(double xVelocity, double yVelocity) {
    fieldCentricSpeeds[0] = xVelocity;
    fieldCentricSpeeds[1] = yVelocity;
  }
}
