/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands.conveyor;
import frc.robot.commands.conveyor.modes.ConveyorSetMode;
import frc.robot.commands.conveyor.modes.ConveyorMode;
import frc.robot.subsystems.Conveyor;



public class ConveyorForceEject extends ConveyorSetMode {

    public ConveyorForceEject(Conveyor conveyor) {
        super(conveyor, ConveyorMode.EJECT);
    }
}
