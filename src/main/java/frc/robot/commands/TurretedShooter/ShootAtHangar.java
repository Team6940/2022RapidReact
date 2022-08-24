package frc.robot.commands.TurretedShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootAtHangar extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final SwerveDriveTrain m_drive;

    public ShootAtHangar(Shooter shooter, Turret turret, SwerveDriveTrain drive) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        m_turret.trackTarget(false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Shooter Running", true);
        m_turret.aimAtGoal(m_drive.getPose(), GoalConstants.kHangerLocation, false); //TODO
        m_shooter.setShooterSpeed(ShooterConstants.kHangarRPM);  //TODO
        m_shooter.setHoodAngle(HoodConstants.kMaxAngle);  //TODO
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        m_turret.trackTarget(false);
        m_turret.On(false);
        m_shooter.setShooterToStop();
    }

}
