package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.leds.LedSubsystem;

public class cmdSetLeds extends CommandBase {

    LedSubsystem ledSubsystem;

    public cmdSetLeds(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ledSubsystem.writePeriodicOutputs();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
