// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.conveyor.modes.*;

/*-
 * The Conveyor subsystem is meant to transport team alliance balls from the intake to the shooter, while rejecting balls that are
 * of the opposing alliance's color.
 *
 * Contains:
 * - A gandalf motor above a gap that will spin a wheel either out the back of the robot or upwards to the upper conveyor
 * - An upper conveyor (with a motor) that can hold a ball or transport balls to the shooter
 * - A color sensor that detects the color of the ball held in the gap near the gandalf motor
 * - An IR Sensor that detects the presence of a ball in the upper conveyor
 *
 
 */
public class Conveyor extends SubsystemBase {

    public enum Direction {
        FORWARD,
        FORWARD_SLOW,
        STOPPED,
        REVERSE
    }

    private ConveyorMode mode;


    private final WPI_TalonFX m_intakermotor;
    private final WPI_TalonFX topBeltMotor;

    private final ColorSensor colorSensor;
    private final DigitalInput topIRSensor;

    private Direction topBeltDirection;
    private Direction gandalfDirection;

    /** Creates a Conveyor subsystem */
    public Conveyor(ColorSensor colorSensor) {
        topBeltMotor = new WPI_TalonFX(Constants.BallLoaderPort);
        m_intakermotor = new WPI_TalonFX(Constants.IntakerPort);
        m_intakermotor.configVoltageCompSaturation(12);
        m_intakermotor.enableVoltageCompensation(true);
        topBeltMotor.configVoltageCompSaturation(12);
        topBeltMotor.enableVoltageCompensation(true);
        this.colorSensor = colorSensor;
        this.topIRSensor = new DigitalInput(Constants.TOP_BALL_IR_SENSOR);

        setTopBelt(Direction.STOPPED);
        setGandalf(Direction.STOPPED);
        setMode(ConveyorMode.DEFAULT);
    }

    /*** MODE CONTROL ***/

    public void setMode(ConveyorMode mode) {
        this.mode = mode;
    }

    /*** MOTOR CONTROL ***/

    /** Spins the Top Conveyor Belt, moving the ball up to the shooter. If false, */
    public void setTopBelt(Direction direction) {
        topBeltDirection = direction;
        switch (direction) {
            case FORWARD:
                topBeltMotor.set(Constants.TOP_BELT_SPEED);
                break;
            case FORWARD_SLOW:
                topBeltMotor.set(
                    Constants.TOP_BELT_SPEED *Constants.SLOW_MUL);
                break;
            case STOPPED:
                topBeltMotor.stopMotor();
                break;
            case REVERSE:
                topBeltMotor.set(Constants.REJECT_SPEED);
                break;
        }
    }

    public void setGandalf(Direction direction) {
        gandalfDirection = direction;
        switch (direction) {
            case FORWARD:
                m_intakermotor.set(Constants.ACCEPT_SPEED);
                break;
            case FORWARD_SLOW:
                m_intakermotor.set(
                    Constants.ACCEPT_SPEED * Constants.SLOW_MUL);
                break;
            case STOPPED:
                m_intakermotor.stopMotor();
                break;
            case REVERSE:
                m_intakermotor.set(Constants.REJECT_SPEED);
                break;
        }
    }

    public Direction getTopBeltDirection() {
        return topBeltDirection;
    }

    public Direction getGandalfDirection() {
        return gandalfDirection;
    }

    /*** SENSOR INFORMATION ***/

    /** Finds if the upper IR Sensor has been tripped e.g., there is a ball in the top conveyor */
    public boolean hasTopBeltBall() {
        return !topIRSensor.get();
    }

    public boolean hasAnyBall() {
        return colorSensor.hasBallOnSensor();
    }

    public boolean hasAllianceBall() {
        return colorSensor.hasAllianceBall();
    }

    public boolean hasOpponentBall() {
        return colorSensor.hasOpponentBall();
    }

    /*** AUTOMATIC RETRACTION ***/

    public boolean isEmpty() {
        return (hasTopBeltBall() || hasAnyBall());
    }

    public boolean isFull() {
        return hasTopBeltBall() && hasAllianceBall();
    }

    /*** PERIODIC COMMAND ***/

    @Override
    public void periodic() {
        mode.run(this);

        SmartDashboard.putNumber("Debug/Conveyor/Top Belt", topBeltMotor.get());
        SmartDashboard.putNumber("Debug/Conveyor/Gandalf Motor", m_intakermotor.get());
        SmartDashboard.putBoolean("Debug/Conveyor/Top IR", hasTopBeltBall());
    }
}
