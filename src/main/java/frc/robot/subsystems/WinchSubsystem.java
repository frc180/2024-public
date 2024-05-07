package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Helpers;
import monologue.Logged;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class WinchSubsystem extends SubsystemBase implements Logged {
    public static final double UP = 16;
    public static final double DOWN = 0;

    private static final double MAX = 16.5 * (9.0 / 8.0);
    private static final double MIN = 0.0;

    private CANSparkMax winchMotor1, winchMotor2;
    private Servo winchServo1, winchServo2;
    private final int winchMotorID1 = 27;
    private final int winchMotorID2 = 28;
    private final int winchServoID1 = 0;
    private final int winchServoID2 = 1;
    private SparkPIDController winchMotor1PID;
    private SparkPIDController winchMotor2PID;
    private RelativeEncoder winchEncoder1, winchEncoder2;
    private DigitalInput winch1Zero, winch2Zero;
    public boolean isClimbing = false;
    public double winch1Target;
    public double winch2Target;
    public boolean isLatched = true;
    private boolean climbEndCurrent = false;
    private Debouncer climbCurrentDebouncer = new Debouncer(0.5);
    public double positionTarget = 0;

    public WinchSubsystem() {
        winchMotor1 = new CANSparkMax(winchMotorID1, MotorType.kBrushless);
        winchMotor1.restoreFactoryDefaults();
        winchMotor1.setIdleMode(IdleMode.kBrake);
        Helpers.setSparkInverted(winchMotor1, true);

        winchMotor2 = new CANSparkMax(winchMotorID2, MotorType.kBrushless);
        winchMotor2.restoreFactoryDefaults();
        winchMotor2.setIdleMode(IdleMode.kBrake);
        winchMotor2.setInverted(false);

        winchEncoder1 = winchMotor1.getEncoder();
        winchEncoder1.setPosition(0);
        winchEncoder2 = winchMotor2.getEncoder();
        winchEncoder2.setPosition(0);

        winchMotor1PID = winchMotor1.getPIDController();
        winchMotor1PID.setP(0.04);
        winchMotor2PID = winchMotor2.getPIDController();
        winchMotor2PID.setP(0.04);

        winchServo1 = new Servo(winchServoID1);
        winchServo2 = new Servo(winchServoID2);

        winch1Zero = new DigitalInput(5);
        winch2Zero = new DigitalInput(4);

        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // motor current
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // motor position
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        winchMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // motor current
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // motor position
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        winchMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    }


    public void periodic() {
        if (winch1Zero()) winchEncoder1.setPosition(0);
        if (winch2Zero()) winchEncoder2.setPosition(0);

        double winch1Current = winchMotor1.getOutputCurrent();
        double winch2Current = winchMotor2.getOutputCurrent();
        climbEndCurrent = climbCurrentDebouncer.calculate(winch1Current >= 20 && winch2Current >= 20);

        log("Climbing?", isClimbing);
        log("Climb End Current?", climbEndCurrent);

        log("Winch1 Position", winchEncoder1.getPosition());
        log("Winch1 Current (A)", winch1Current);

        log("Winch2 Position", winchEncoder2.getPosition());
        log("Winch2 Current (A)", winch2Current);

        log("Winch1 Zero?", winch1Zero());
        log("Winch2 Zero?", winch2Zero());
    }

    public boolean winch1Zero() {
        return !winch1Zero.get();
    }

    public boolean winch2Zero() {
        return !winch2Zero.get();
    }

    public boolean isUp() {
        return positionTarget == UP;
    }

    public boolean shouldClimbEnd() {
        return winch1Zero() && winch2Zero();
    }

    public void stop() {
        winchMotor1.set(0);
        winchMotor2.set(0);
    }

    public void setWinchPosition(double position) {
        winch1Target = boundPosition(position);
        winch2Target = boundPosition(position);
        positionTarget = position;
        winchMotor1PID.setReference(winch1Target, ControlType.kPosition);
        winchMotor2PID.setReference(winch2Target, ControlType.kPosition);
    }

    public double boundPosition(double input) {
        if(input >= MAX){
            return MAX;
        } else if(input <= MIN){
            return MIN;
        } else{
            return input;
        }
    }

    public double getWinch1Target() {
        return winch1Target;
    }

    public double getWinch2Target() {
        return winch2Target;
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    public void setClimbing(boolean climb){
        isClimbing = climb;
    }

    public void setServos(double angle) {
        if (angle > 90) angle = 90;
        if (angle < 0) angle = 0;
        winchServo1.setAngle(angle);
        winchServo2.setAngle(90 - angle);
    }

    public Command setWinchMotorOneSpeed(double speed) {
        return runEnd(
            () -> {
                winchMotor1.set(speed);
            },
            () -> {
                stop();
            }
        );
    }

    public Command setWinchMotorTwoSpeed(double speed) {
        return runEnd(
            () -> {
                winchMotor2.set(speed);
            },
            () -> {
                stop();
            }
        );
    }

    public Command setWinchSpeed(double speed) {
        return runEnd(
            () -> {
                winchMotor1.setVoltage(speed * 12);
                winchMotor2.setVoltage(speed * 12);
            },
            () -> {
                stop();
            }
        );
    }

    public Command runWinchToZero(double speed, boolean latch) {
        Command toZeroCommand = runEnd(() -> {
            positionTarget = 0;
            if (winch1Zero()) {
                winchMotor1.set(0);
            } else {
                winchMotor1.set(speed);
            }
            if (winch2Zero()) {
                winchMotor2.set(0);
            } else {
                winchMotor2.set(speed);
            }
        }, 
        () -> {
            stop();
        }).until(() -> winch1Zero() && winch2Zero());

        return Commands.sequence(
            setServoLatchSafe(latch),
            toZeroCommand
        );
    }


    public Command setServosCommand(double angle) {
        return Commands.runOnce(() -> {
            setServos(angle);
            isLatched = angle == 0;
        });
    }

    public Command servoLatch() {
        return setServosCommand(0);
    }

    public Command servoUnlatch() {
        return setServosCommand(90);
    }

    public Command setServoLatch(boolean latch) {
        return latch ? servoLatch() : servoUnlatch();
    }

    public Command setServoLatchSafe(boolean latch) {
        Command latchCommand = setServoLatch(latch).andThen(Commands.waitSeconds(0.3));
        return Commands.either(latchCommand, Commands.none(), ()-> isLatched != latch);
    }
}
