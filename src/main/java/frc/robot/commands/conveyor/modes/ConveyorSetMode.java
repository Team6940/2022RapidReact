/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands.conveyor.modes;

import frc.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ConveyorSetMode extends CommandBase {

    protected final ConveyorMode mode;
    protected final Conveyor conveyor;

    /** Creates a new ConveyorIndexCommand. */
    protected ConveyorSetMode(Conveyor conveyor, ConveyorMode mode) {
        this.conveyor = conveyor;
        this.mode = mode;

        addRequirements(conveyor);
    }

    @Override
    public final void execute() {
        conveyor.setMode(this.mode);
    }

    @Override
    public final void end(boolean interrupted) {
        conveyor.setMode(ConveyorMode.DEFAULT);
    }
}
