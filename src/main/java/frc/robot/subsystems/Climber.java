package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.ClimberConstants;
import frc.robot.lib.team3476.Timer;
//import frc.utility.controllers.LazyTalonSRX;
//import org.jetbrains.annotations.NotNull;

import java.util.Objects;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.util.function.Function;

import static frc.robot.Constants.ClimberConstants.CLIMBER_ENCODER_TICKS_PER_INCH;
import static frc.robot.Constants.ClimberConstants.DO_BACK_HOOK_CLIMB;
//import static frc.utility.Pneumatics.getPneumaticsHub;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;

class ClimbStep {
    /**
     * The action to perform when the state is entered.
     */
    final Consumer<Climber> startAction;

    /**
     * The condition to wait for before transitioning to the next state.
     * <p>
     * Returns true when the condition is met. (When true is returned, the climber state should be transitioned to the next
     * state.) aka (true = continue, false = wait)
     */
    final Function<Climber, Boolean> waitCondition;

    /**
     * The action to perform when the state is exited.
     */
    final Consumer<Climber> endAction;

    /**
     * true if this step causes something on the robot to move
     */
    final boolean runInStepByStep;

    /**
     * @param startAction     The action to perform when the state is entered
     * @param waitCondition   The condition to wait for before transitioning to the next state (true = continue, false = wait)
     * @param endAction       The action to perform when the state is exited
     * @param runInStepByStep if this step causes something on the robot to move
     */
    ClimbStep(Consumer<Climber> startAction, Function<Climber, Boolean> waitCondition, Consumer<Climber> endAction,
              boolean runInStepByStep) {
        this.startAction = startAction;
        this.waitCondition = waitCondition;
        this.endAction = endAction;
        this.runInStepByStep = runInStepByStep;
    }
}

public final class Climber extends SubsystemBase {

    private static Climber instance = null;


    // Safe Lazy Initialization. Initializes itself when first called
    public static Climber getInstance() {
        if (instance == null){
            instance = new Climber();
        }
        return instance;
    }

    private final  WPI_TalonFX climberMotor;
    private final  WPI_TalonFX climberMotor2;

    private final  DigitalInput elevatorArmContactSwitchA; //表示elevator前排挂钩是否挂到拉杆
    private final  DigitalInput elevatorArmContactSwitchB; //表示elevator前排挂钩是否挂到拉杆

    private final  DigitalInput pivotingArmContactSwitchA; //表示后排挂钩是否挂到拉杆
    private final  DigitalInput pivotingArmContactSwitchB; //表示后排挂钩是否挂到拉杆
    //private final  DigitalInput pivotingArmLatchedSwitchA;
    //private final  DigitalInput pivotingArmLatchedSwitchB;

    //private final  Solenoid latchSolenoid; /* 控制抓手,我们不用 */
    private final  Solenoid pivotSolenoid; /* 两侧控制机器人倾斜 */
    //private final  Solenoid brakeSolenoid; /* 控制elevator刹住不动,我们不用*/


    private double data;

    private Climber.ClimbState climbState = ClimbState.IDLE;

    private boolean isPaused = false;
    private double pausedClimberSetpoint;
    //private BrakeState pausedBrakeState = BrakeState.BRAKING;
    private ControlMode pausedClimberMode;
    private boolean stepByStep = false;
    private boolean advanceStep = false;
    private boolean skipChecks = false;
    private boolean ranEndAction = false;
    private boolean startingClimb = true;

    //public enum ClawState {
    //    LATCHED, UNLATCHED
    //}

    enum PivotState {
        PIVOTED, INLINE
    }

    //public enum BrakeState {
    //    BRAKING, FREE
    //}

    //public void setBrakeState(BrakeState brakeState) {
    //    brakeSolenoid.set(brakeState == BrakeState.FREE);
    //}

    //public void setClawState(ClawState clawState) {
        //latchSolenoid.set(clawState == ClawState.UNLATCHED);
    //}

    public void setPivotState(PivotState pivotState) {
        pivotSolenoid.set(pivotState == PivotState.PIVOTED);
    }

    //public BrakeState getBrakeState() {
    //    return brakeSolenoid.get() ? BrakeState.FREE : BrakeState.BRAKING;
    //}

    //public ClawState getClawState() {
    //    return latchSolenoid.get() ? ClawState.UNLATCHED : ClawState.LATCHED;
    //}

    public PivotState getPivotState() {
        return pivotSolenoid.get() ? PivotState.PIVOTED : PivotState.INLINE;
    }

    public enum ClimbState {
        /**
         * The climb state when we're not climbing.
         */
        IDLE(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> false,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Waits till the conditions are right to start the climb. Note: we run the same climb sequence twice. The first time it
         * runs we end up at the high bar and the second time we run we end up at the traversal bar.
         */
        START_CLIMB(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            // If this is the first time we're running the climb sequence, we want to immediately start the
                            // climb. (This is after the robot has deployed the climber elevator and the robot is still on the
                            // ground)
                            if (cl.startingClimb) return true;

                            // Check that we're in the right part of our swing to continue the climb. We should be in the middle
                            // of the swing and be moving towards the field. Climbing at this point gives the robot the least
                            // amount of rotational energy.
                            double Roll = SwerveDriveTrain.getInstance().getRoll();
                            return (Roll < 10 && Roll > -10 && RobotTracker.getInstance().getGyroRollVelocity() < 0); //TODO 暂时删除
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * Lowers the elevator arm until the contact switch is pressed on the bar.
         */
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                new ClimbStep(
                        (cl) -> {
                            cl.startingClimb = false;
                            //cl.setBrakeState(BrakeState.FREE);
                            cl.climberMotor.set(ControlMode.MotionMagic, ClimberConstants.CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION);//TODO Change value the extension
                            //cl.setClawState(ClawState.UNLATCHED);
                        },
                        (cl) -> Math.abs(
                                ClimberConstants.CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION - cl.climberMotor.getSelectedSensorPosition())
                                < ClimberConstants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Extends the solenoid to latch the pivoting arm (one with the pneumatic claws) onto the bar. Waits until the latch
         * switch is pressed.
         */
        LATCH_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {},
                        //(cl) -> cl.setClawState(ClawState.LATCHED),
                        Climber::isPivotingArmLatched,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Moves the elevator arm up to the maximum safe height. This is the height at which the elevator arm won't contact
         *     the next bar. Will continue to the next state once the elevator is no longer supporting the robot. (Only the
         *     pivoting arms are supporting the entire robot)
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Moves the elevator arm up to the maximum height. At this height the arm will contact the next bar when the robot
         *     unpivots. Will continue to the next state once the elevator arm is at it's Max Extension
         *     ({@link ClimberConstants#MAX_CLIMBER_EXTENSION}). At this point the robot is only supporting the pivoting arm.
         * </ul>
         */
        MOVE_WEIGHT_TO_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            if (cl.shouldDoBackHookStep()) {
                                cl.data = ClimberConstants.MAX_CLIMBER_EXTENSION;
                                cl.climberMotor.set(ControlMode.MotionMagic, ClimberConstants.MAX_CLIMBER_EXTENSION);
                            } else {
                                cl.data = ClimberConstants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                                cl.climberMotor.set(ControlMode.MotionMagic, ClimberConstants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                            }
                        },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > cl.data - ClimberConstants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Extends the solenoid to pivot the robot and cause it to start swinging. Waits a minimum time and until the climber
         *     motor has fully extended to it's maximum safe position.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Do Nothing
         * </ul>
         */
        PIVOT_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            if (!cl.shouldDoBackHookStep()) {
                                cl.setPivotState(PivotState.PIVOTED);
                            }
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> {
                            if (cl.shouldDoBackHookStep()) return true; // We didn't pivot, continue
                            return Timer.getFPGATimestamp() - cl.data > ClimberConstants.ARM_PIVOT_DURATION; // true if we ARM_PIVOT_DURATION time has elapsed
                        },
                        (cl) -> {},
                        true
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Uses the gyro to wait until the robot is swinging towards the field and it's safe to extend the elevator arm. At
         *     this point we know that the elevator arm won't touch the next bar when it extends.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Do Nothing. The timing with the swing works out so that we'll be at correct angle to swing into the bar if we
         *     just immediately continue and pivot the robot.
         * </ul>
         */
        WAIT_TILL_EXTENSION_IS_SAFE(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            double Roll = SwerveDriveTrain.getInstance().getRoll();
                            if (cl.shouldDoBackHookStep()) {
                                return true;
                            } else {
                                return RobotTracker.getInstance().getGyroRollVelocity() > 0
                                        && Roll > ClimberConstants.ELEVATOR_ARM_SAFE_ANGLE;  //TODO  暂时删除getGyroRollVelocity
                            }
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Extends the elevator arm past the safe height. At this point the arm will contact the bar when the robot swings
         *     back towards the bars.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Skip this step. The arm has already been commanded to extend past the safe height.
         * </ul>
         */
        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                new ClimbStep(
                        (cl) -> {
                            if (!cl.shouldDoBackHookStep()) {
                                cl.climberMotor.set(ControlMode.MotionMagic, ClimberConstants.MAX_CLIMBER_EXTENSION);
                            }
                        },
                        (cl) -> {
                            if (cl.timesRun == 1) return true;
                            return cl.climberMotor.getSelectedSensorPosition() > ClimberConstants.MAX_CLIMBER_EXTENSION - ClimberConstants.CLIMBER_MOTOR_MAX_ERROR;
                        },
                        (cl) -> {},
                        true
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Unpivot the elevator arm so elevator arm is pulled onto the next bar. Waits
         *     {@link ClimberConstants#ARM_UNPIVOT_DURATION} for the arm to unpivot.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Pivot the arm towards the next bar. Once the arm contacts the next bar, the robot will have latched onto the
         *     next bar. The timing works out so that the robot will be at the correct part of the swing to latch onto the next
         *     bar.
         * </ul>
         */
        CONTACT_NEXT_BAR(
                new ClimbStep(
                        (cl) -> {
                            if (cl.shouldDoBackHookStep()) {
                                // Don't unpivot if we're using back hooks
                                cl.setPivotState(PivotState.PIVOTED);
                                cl.data = Timer.getFPGATimestamp();
                            } else {
                                cl.setPivotState(PivotState.INLINE);
                                cl.data = Timer.getFPGATimestamp() + ClimberConstants.ARM_UNPIVOT_DURATION;
                            }
                        },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > 0,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Waits for the gyro to report that the robot has reached the bar.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Stop the climb from continuing. Wait for the operator to force advance the climb to unlatch the robot from the bar.
         * </ul>
         */
        WAIT_FOR_SWING_STOP(
                new ClimbStep(
                        (cl) -> cl.data = Double.MAX_VALUE,
                        (cl) -> {
                            double Roll = SwerveDriveTrain.getInstance().getRoll();
                            if (cl.shouldDoBackHookStep()) {
                                return false;
                            }
                            return Roll < ClimberConstants.ON_NEXT_BAR_ANGLE;
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * On the first run or when the back hook climb is disabled:
         * <ul>
         *     Retracts the elevator arm so that it contacts the next bar (and squeezes it a bit). This ensures that the robot
         *     will remain on the next bar after unlatching from the previous bar.
         * </ul>
         * When doing the back hook climb:
         * <ul>
         *     Bring the elevator arm down a bit to ensure that the robot doesn't drop much after unlatching from the previous bar.
         * </ul>
         */
        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                new ClimbStep(
                        (cl) -> {
                            if (cl.shouldDoBackHookStep()) {
                                cl.climberMotor.set(ControlMode.MotionMagic,
                                        ClimberConstants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_BACK_HOOK);
                                cl.data = ClimberConstants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_BACK_HOOK;
                            } else {
                                cl.climberMotor.set(ControlMode.MotionMagic,
                                        ClimberConstants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_FRONT_HOOK);
                                cl.data = ClimberConstants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION_FRONT_HOOK;
                            }
                            SwerveDriveTrain.getInstance().setSWERVE_MODULE_STATE_FORWARD();
                        },
                        (cl) -> Math.abs(cl.climberMotor.getSelectedSensorPosition() - cl.data)
                                < (0.3 * CLIMBER_ENCODER_TICKS_PER_INCH),
                        (cl) -> {
                            cl.stopClimberMotor();
                            //cl.setBrakeState(BrakeState.BRAKING);
                        },
                        true
                )
        ),

        /**
         * Wait a short amount of time to ensure that the break is engaged.
         */
        WAIT_FOR_BRAKE_TIME(
                new ClimbStep(
                        (cl) -> cl.data = Timer.getFPGATimestamp(),
                        (cl) -> {
                            if (Timer.getFPGATimestamp() - cl.data > 0.25) {
                                cl.climberMotor.set(ControlMode.PercentOutput, 0);
                                return true;
                            }
                            return false;
                        },
                        (cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0),
                        true
                )
        ),

        /**
         * Unlatches the pivot arm so that the robot is supported by the elevator arm and is on the next bar.
         */
        UNLATCH_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            //cl.setClawState(ClawState.UNLATCHED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > ClimberConstants.PIVOT_ARM_UNLATCH_DURATION,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * If we're using back hooks, we need to move the pivot arm back into the robot after we unlatch it.
         */
        UNPIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            cl.setPivotState(PivotState.INLINE);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> {
                            if (cl.timesRun != 1 || !DO_BACK_HOOK_CLIMB) {
                                return true; //We've already unpivoted, so we don't need to wait
                            }
                            return Timer.getFPGATimestamp() - cl.data > ClimberConstants.ARM_UNPIVOT_DURATION;
                        },
                        (cl) -> {},
                        true
                )
        );

        // At this point the climb will either stop or go back to the START_CLIMB step (depending on how many loops
        // we've already done).

        /**
         * @param climbStep The climb step to use when doing a manual climb, step by step
         */
        ClimbState(ClimbStep climbStep) {
            this.climbStep = climbStep;
        }

        /**
         * The climb step to use when doing a manual climb, step by step
         */
        final ClimbStep climbStep;
    }

    private boolean shouldDoBackHookStep() {
        return this.timesRun == 1 && DO_BACK_HOOK_CLIMB;
    }

    double otherPivotingArmMustContactByTime = Double.MAX_VALUE;

    /**
     * Will pause the climb if it detects that only one pivot arm is in contact with the bar for some time.
     *
     * @return true if both pivot arms are in contact with the bar
     */
    private boolean isPivotArmContactingBar() {
        if (pivotingArmContactSwitchA.get() ^ pivotingArmContactSwitchB.get()) {
            if (otherPivotingArmMustContactByTime > Timer.getFPGATimestamp() + ClimberConstants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME) {
                otherPivotingArmMustContactByTime = Timer.getFPGATimestamp() + ClimberConstants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME;
            } else if (Timer.getFPGATimestamp() > otherPivotingArmMustContactByTime) {
                pauseClimb();
            }
        } else if (pivotingArmContactSwitchA.get() && pivotingArmContactSwitchB.get()) {
            return true;
        } else {
            otherPivotingArmMustContactByTime = Double.MAX_VALUE;
        }
        return false;
    }

    double otherElevatorArmMustContactByTime = Double.MAX_VALUE;

    /**
     * Will pause the climb if it detects that only one pivot arm is in contact with the bar for some time.
     *
     * @return true if both pivot arms are in contact with the bar
     */
    private boolean isElevatorArmContactingBar() {
        if (elevatorArmContactSwitchA.get() ^ elevatorArmContactSwitchB.get()) {
            if (otherElevatorArmMustContactByTime > Timer.getFPGATimestamp() + ClimberConstants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME) {
                otherElevatorArmMustContactByTime = Timer.getFPGATimestamp() + ClimberConstants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME;
            } else if (Timer.getFPGATimestamp() > otherElevatorArmMustContactByTime) {
                pauseClimb();
            }
        } else if (elevatorArmContactSwitchA.get() && elevatorArmContactSwitchB.get()) {
            return true;
        } else {
            otherElevatorArmMustContactByTime = Double.MAX_VALUE;
        }
        return false;
    }

    private int timesRun;

    private Climber() {
        //super(ClimberConstants.CLIMBER_PERIOD, 1);

        climberMotor = new WPI_TalonFX(ClimberConstants.leftClimberMotorPort);
        climberMotor2 =  new WPI_TalonFX(ClimberConstants.rghtClimberMotorPort);
        climberMotor2.follow(climberMotor);
        climberMotor.config_kF(0, ClimberConstants.CLIMBER_MOTOR_KF);
        climberMotor.config_kP(0, ClimberConstants.CLIMBER_MOTOR_KP);
        climberMotor.config_kI(0, ClimberConstants.CLIMBER_MOTOR_KI);
        climberMotor.config_kD(0, ClimberConstants.CLIMBER_MOTOR_KD);
        climberMotor.config_IntegralZone(0, ClimberConstants.CLIMBER_MOTOR_IZONE);
        climberMotor.configMaxIntegralAccumulator(0, ClimberConstants.CLIMBER_MOTOR_MAX_IACCUMULATOR);
        climberMotor.configPeakOutputForward(ClimberConstants.CLIMBER_MOTOR_MAX_OUTPUT);
        climberMotor.configPeakOutputReverse(-ClimberConstants.CLIMBER_MOTOR_MAX_OUTPUT);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ClimberConstants.CLIMBER_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_CURRENT_LIMIT, 0));
        climberMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ClimberConstants.CLIMBER_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_CURRENT_LIMIT, 0));

        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20); // Default is 10ms
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 25); // Default is 10ms
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        climberMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        climberMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        climberMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        climberMotor.configMotionAcceleration(15 * CLIMBER_ENCODER_TICKS_PER_INCH);
        climberMotor.configMotionCruiseVelocity(48 * CLIMBER_ENCODER_TICKS_PER_INCH);
        climberMotor.configClosedloopRamp(0);

        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100); // Default is 10ms
        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100); // Default is 10ms
        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        climberMotor2.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        climberMotor2.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        climberMotor2.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        elevatorArmContactSwitchA = new DigitalInput(ClimberConstants.ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        elevatorArmContactSwitchB = new DigitalInput(ClimberConstants.ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);

        pivotingArmContactSwitchA = new DigitalInput(ClimberConstants.PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        pivotingArmContactSwitchB = new DigitalInput(ClimberConstants.PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);
        //pivotingArmLatchedSwitchA = new DigitalInput(ClimberConstants.PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL);
        //pivotingArmLatchedSwitchB = new DigitalInput(ClimberConstants.PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL);

        //latchSolenoid = getPneumaticsHub().makeSolenoid(ClimberConstants.LATCH_SOLENOID_ID);
        //pivotSolenoid = getPneumaticsHub().makeSolenoid(ClimberConstants.PIVOT_SOLENOID_ID);
        //brakeSolenoid = getPneumaticsHub().makeSolenoid(ClimberConstants.BRAKE_SOLENOID_ID);
        
        pivotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.PIVOT_SOLENOID_ID);

        climberMotor.setInverted(true);
        climberMotor2.setInverted(true);

        climberMotor.setSelectedSensorPosition(0);

        timesRun = 0;
    }

    /**
     * Starts the automated climb sequence and deactivates the brake.
     */
    public synchronized void startClimb() {
        isPaused = false;
        otherPivotingArmMustContactByTime = Double.MAX_VALUE;
        climbState = ClimbState.START_CLIMB;
        startingClimb = true;
        //setBrakeState(BrakeState.FREE);
    }

    /**
     * Stops the climb and resets the climber state. Also activates the brake.
     */
    public synchronized void stopClimb() {
        climbState = ClimbState.IDLE;
        climberMotor.set(ControlMode.PercentOutput, 0);
        //setBrakeState(BrakeState.BRAKING);
        timesRun = 0;
        hasStalledIntoBottom = false;
        minRunTime = -1;
    }

    /**
     * Pauses the climb.
     * <p>
     * It first stores the current state of the climber in a variable, then sets stops the climber and activates the brake.
     */
    public synchronized void pauseClimb() {
        isPaused = true;
        if (!RobotBase.isSimulation()) {
            pausedClimberMode = climberMotor.getControlMode();
        }else{
            pausedClimberMode = ControlMode.PercentOutput;
        }
        if (pausedClimberMode == ControlMode.PercentOutput) {
            pausedClimberSetpoint = climberMotor.getMotorOutputPercent();
        } else {
            pausedClimberSetpoint = climberMotor.getClosedLoopTarget();
        }
        //pausedBrakeState = getBrakeState();
        climberMotor.set(ControlMode.PercentOutput, 0);
        //setBrakeState(BrakeState.BRAKING);
    }

    /**
     * Stops the climber motor from moving.
     * <p>
     * It sets the motor to Position Control mode and sets the setpoint to the current position.
     */
    private synchronized void stopClimberMotor() {
        climberMotor.set(ControlMode.MotionMagic, climberMotor.getSelectedSensorPosition());
    }

    /**
     * Resumes the climber from a paused state.
     */
    public synchronized void resumeClimb() {
        isPaused = false;
        otherPivotingArmMustContactByTime = Double.MAX_VALUE;
        //setBrakeState(pausedBrakeState);
        climberMotor.set(pausedClimberMode, pausedClimberSetpoint);
    }

    /**
     * Sets the climber in the correct state to initiate a climb and moves the elevator arm to the up above the high bar.
     */
    public synchronized void deployClimb() {
        timesRun = 0;
        climberMotor.set(ControlMode.MotionMagic, ClimberConstants.CLIMBER_DEPLOY_HEIGHT);
        //setBrakeState(BrakeState.FREE);
        //setClawState(ClawState.UNLATCHED);
        setPivotState(PivotState.INLINE);
        startingClimb = true;
    }

    /**
     * Tells the robot to advance the climber to the next state in step by step mode. The robot will still wait till all checks
     * are passing before advancing.
     * <p>
     * Step-by-step mode must be enabled for this method to have an effect.
     */
    public synchronized void advanceStep() {
        advanceStep = true;
    }

    /**
     * Forces the robot to move to the next step immediately. No checks are made to ensure that the robot is in the correct state
     * to move to the next step. <b> The person calling this method needs to ensure that the robot is in the correct state.</b>
     * <p>
     * This method will also work regardless of whether the robot is in step-by-mode or not.
     */
    public synchronized void forceAdvanceStep() {
        if (isPaused) resumeClimb();
        advanceStep = true;
        skipChecks = true;
    }

    public synchronized void setStepByStep(boolean stepByStep) {
        this.stepByStep = stepByStep;
    }

    public synchronized boolean isStepByStep() {
        return stepByStep;
    }

    @Override
    public void periodic() {
        ClimbStep currentClimbStep = climbState.climbStep;

        if (Timer.getFPGATimestamp() > resetZeroAtTime) {
            climberMotor.setSelectedSensorPosition(0);
            resetZeroAtTime = Double.MAX_VALUE;
        }

        if (!isPaused) {
            //            if (climberMotor.getSelectedSensorPosition() < ClimberConstants.MIN_CLIMBER_ELEVATOR_HEIGHT
            //                    && climberMotor.getSelectedSensorVelocity() < 0) {
            //                stopClimb();
            //            } else if (climberMotor.getSelectedSensorPosition() > ClimberConstants.MAX_CLIMBER_ELEVATOR_HEIGHT
            //                    && climberMotor.getSelectedSensorVelocity() > 0) {
            //                stopClimb();
            //            }

            if (currentClimbStep.waitCondition.apply(this)) {
                currentClimbStep.endAction.accept(this);
                ranEndAction = true;
            }

            if (skipChecks || ((currentClimbStep.waitCondition.apply(this)) && (!stepByStep || advanceStep))) {
                if (!ranEndAction)
                    currentClimbStep.endAction.accept(this);
                do {
                    climbState = ClimbState.values()[(climbState.ordinal() + 1) % ClimbState.values().length];
                    if (climbState == ClimbState.IDLE && timesRun < 1) {
                        climbState = ClimbState.START_CLIMB;
                        timesRun++;
                    }

                    currentClimbStep = climbState.climbStep;
                } while (!currentClimbStep.runInStepByStep && stepByStep);

                currentClimbStep.startAction.accept(this);
                advanceStep = false;
                skipChecks = false;
                ranEndAction = false;
            }
        }
        outputTelemetry();
    }

    /**
     * @param percentOutput The percent output to set the climber motor to. Will automatically activate/deactivate the brake
     */
    public void setClimberMotor(double percentOutput) {
        isPaused = true;
        climbState = ClimbState.IDLE;
        //setBrakeState(Math.abs(percentOutput) < 1.0E-2 ? BrakeState.BRAKING : BrakeState.FREE);
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    private boolean hasStalledIntoBottom = false;
    private double minRunTime = -1;
    private double resetZeroAtTime = Double.MAX_VALUE;

    /**
     * moves the climber down until it stall at the bottom. Press reset button to run again.
     */
    public synchronized void stallIntoBottom() {
        if (minRunTime == -1) minRunTime = Timer.getFPGATimestamp() + 0.5;
        if (hasStalledIntoBottom) {
            setClimberMotor(0);
        } else {
            setClimberMotor(-0.1);
        }

        if (Math.abs(climberMotor.getStatorCurrent()) > 12 && Timer.getFPGATimestamp() > minRunTime) {
            hasStalledIntoBottom = true;
            climberMotor.setSelectedSensorPosition(0);
            resetZeroAtTime = Timer.getFPGATimestamp() + 1;
            setClimberMotor(0);
        }
    }

    /**
     * Toggles the latch that is on the pivot arm.
     */
    public void toggleClaw() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        //setClawState(getClawState() == ClawState.UNLATCHED ? ClawState.LATCHED : ClawState.UNLATCHED);
    }

    /**
     * Toggles the pivot arm in and out. Also pauses the climber motor.
     */
    public void togglePivot() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        setPivotState(getPivotState() == PivotState.INLINE ? PivotState.PIVOTED : PivotState.INLINE);
    }


    public Climber.ClimbState getClimbState() {
        return climbState;
    }

    public boolean isPivotingArmLatched() {
        return true;  //TODO 
        //we havn't two sensor, so always true;
        //return isPivotingArmLatchedSwitchA() && isPivotingArmLatchedSwitchB();
    }

    //public boolean isPivotingArmLatchedSwitchA() {
    //    return pivotingArmLatchedSwitchA.get();
    //}

    //public boolean isPivotingArmLatchedSwitchB() {
    //    return pivotingArmLatchedSwitchB.get();
    //}

    public void outputTelemetry(){
        SmartDashboard.putNumber("Debug/Climber/Motor Position", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Debug/Climber/Motor Position IN", climberMotor.getSelectedSensorPosition() / CLIMBER_ENCODER_TICKS_PER_INCH);
        SmartDashboard.putNumber("Debug/Climber/Motor Velocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Debug/Climber/Motor Percent Output", climberMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Debug/Climber/Motor Current", climberMotor.getStatorCurrent());
        SmartDashboard.putNumber("Debug/Climber/Motor 2 Current", climberMotor2.getStatorCurrent());
        SmartDashboard.putNumber("Debug/Climber/Motor Current Limit", ClimberConstants.CLIMBER_CURRENT_LIMIT);

        SmartDashboard.putBoolean("Debug/Climber/Elevator Arm Contact Switch A", elevatorArmContactSwitchA.get());
        SmartDashboard.putBoolean("Debug/Climber/Elevator Arm Contact Switch B", elevatorArmContactSwitchB.get());
        SmartDashboard.putBoolean("Debug/Climber/Pivot Arm Contact Switch A", pivotingArmContactSwitchA.get());
        SmartDashboard.putBoolean("Debug/Climber/Pivot Arm Contact Switch B", pivotingArmContactSwitchB.get());
        //SmartDashboard.putBoolean("Debug/Climber/Pivoting Arm Contact Switch A", isPivotingArmLatchedSwitchA());
        //SmartDashboard.putBoolean("Debug/Climber/Pivoting Arm Contact Switch B", isPivotingArmLatchedSwitchB());
        SmartDashboard.putBoolean("Debug/Climber/Pivoting Arm Latched", isPivotingArmLatched());

        SmartDashboard.putString("Debug/Climber/Pivot Solenoid State", getPivotState().toString());
        //SmartDashboard.putString("Debug/Climber/Latch Solenoid State", getClawState().toString());
        //SmartDashboard.putString("Debug/Climber/Brake Solenoid State", getBrakeState().toString());

        SmartDashboard.putBoolean("Debug/Climber/Is Paused", isPaused);
        SmartDashboard.putBoolean("Debug/Climber/Is Step By Step", stepByStep);
        SmartDashboard.putString("Debug/Climber/Current Climber State", climbState.toString());
        SmartDashboard.putNumber("Debug/Climber/Climb Times Run", timesRun);

        SmartDashboard.putBoolean("Debug/Climber/Current Climber WaitCondition", climbState.climbStep.waitCondition.apply(this));

        SmartDashboard.putBoolean("Debug/Climber/Starting Climb", startingClimb);
    }

    public void configCoast() {
        climberMotor.setNeutralMode(NeutralMode.Coast);
        climberMotor2.setNeutralMode(NeutralMode.Coast);
        //setBrakeState(BrakeState.FREE);
    }

    public void configBrake() {
        climberMotor.setNeutralMode(NeutralMode.Coast);
        climberMotor2.setNeutralMode(NeutralMode.Coast);
        //setBrakeState(BrakeState.BRAKING);
    }


    public boolean isPaused() {
        return isPaused;
    }

    public void autoStartClimb(){
        if (this.getClimbState() == ClimbState.IDLE) {
            this.startClimb();
        } else {
            this.resumeClimb();
            this.advanceStep();
        }
    }
}
