package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Helpers;
import frc.robot.utils.PIDTuner;
import monologue.Logged;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem extends SubsystemBase implements Logged {
    public static int shotNumber = 1;

    private TalonFX intakeMotor;
    private CANSparkMax sweeperMotor;
    private TalonFX backIntakeMotor;
    private DigitalInput frontMid, backMid;
    private Trigger noteDetected;
    private VoltageOut voltageControl;
    private VelocityVoltage velocityControl;
    private PIDTuner pidTuner = null;

    public final int intakeMotorID = 23;
    private final int sweeperMotorID = 24;
    private final int backIntakeMotorID = 32;
    private final int frontMidBeamID = 9;
    private final int backMidBeamID = 8;
    private final double closedLoopRampRate = 0.1;
    private final int smartCurrentLimit = 20;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(intakeMotorID, TunerConstants.kCANbusName);
        intakeMotor.setInverted(true);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(70) // was 80
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(70) // was 80
                                                .withSupplyCurrentLimitEnable(true);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0.12;

        var config = intakeMotor.getConfigurator();
        config.apply(currentLimits);
        config.apply(slot0Configs);

        // pidTuner = new PIDTuner(slot0Configs, intakeMotor, () -> targetVelocity)
        //                 .withName("Intake")
        //                 .withValue(() -> intakeMotor.getVelocity().getValueAsDouble())
        //                 .withOutput(() -> intakeMotor.getDutyCycle().getValueAsDouble());
        // pidTuner.initializeValues(slot0Configs);

        sweeperMotor = new CANSparkMax(sweeperMotorID, MotorType.kBrushless);
        Helpers.setSparkInverted(sweeperMotor, true);
        sweeperMotor.setIdleMode(IdleMode.kCoast);
        sweeperMotor.setClosedLoopRampRate(closedLoopRampRate);
        sweeperMotor.setSmartCurrentLimit(smartCurrentLimit);

        backIntakeMotor = new TalonFX(backIntakeMotorID, TunerConstants.kCANbusName);
        backIntakeMotor.setNeutralMode(NeutralModeValue.Coast);

        config = backIntakeMotor.getConfigurator();
        config.apply(currentLimits);

        frontMid = new DigitalInput(frontMidBeamID);
        backMid = new DigitalInput(backMidBeamID);

        voltageControl = new VoltageOut(0)
                            .withEnableFOC(true);
        velocityControl = new VelocityVoltage(0)
                            .withSlot(0)
                            .withEnableFOC(true);

        noteDetected = new Trigger(this::isNoteDetected);
    }

    public void initialize() {
        stop();
    }

    public void stop() {
        intakeMotor.stopMotor();
        sweeperMotor.stopMotor();
        backIntakeMotor.stopMotor();
    }

    public void intakeSetSpeed(double speed) {
        intakeMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    public void intakeSetVelocity(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    public void setBackIntakeSpeed(double speed) {
        backIntakeMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    public void setSweeper(double speed) {
        sweeperMotor.set(speed);
    }

    public boolean frontMidBreak() {
        return !frontMid.get();
    }

    public boolean backMidBreak() {
        return !backMid.get();
    }

    public boolean isNoteDetected() {
        return frontMidBreak() && backMidBreak();
    }

    public boolean isNotePartialDetected() {
        return frontMidBreak() || backMidBreak();
    }

    public boolean isNoteHalfDetected() {
        return frontMidBreak() != backMidBreak();
    }

    public Trigger noteDetected() {
        return noteDetected;
    }

    public Command reindexCommand() {
        return Commands.sequence(
            setIntakeSpeedCommand(-0.1).withTimeout(2),
            new IntakeCommand(this, true, 0.8).withEndOnNoteDetected(true)
        );
    }

    private Command coreFeedCommand() {
        if (Robot.isReal()) {
            return Commands.deadline(
                Commands.waitUntil(() -> !backMidBreak()),
                setIntakeSpeedCommand(1)
            );
        } else {
            return setIntakeSpeedCommand(1).withTimeout(0.2);
        }
    }

    public Command feedShooterCommand() {
        return Commands.parallel(
            coreFeedCommand(),
            Commands.runOnce(() -> {
                RobotContainer robot = RobotContainer.instance;
                String prefix = "Shots/Shot " + shotNumber + "/";
                log(prefix + "Speaker Angle", robot.vision.getSpeakerAngle());
                log(prefix + "Speaker Distance", robot.vision.getSpeakerDistanceIgnoreOverride());
                log(prefix + "Speaker Angle Error", robot.drivetrain.getSpeakerError());
                log(prefix + "Speaker Tag Height", robot.vision.getSpeakerTagHeight());
                log(prefix + "Shooter Angle", robot.shooterPivot.getCurrentPostion());
                log(prefix + " Time", Timer.getFPGATimestamp() - Robot.autoStartTime);
                shotNumber += 1;
            })
        );
    }

    public Command setIntakeSpeedCommand(double speed) {
        return runEnd(
            ()-> {
                intakeSetSpeed(speed);
            },
            () -> {
                intakeMotor.set(0);
            }
        );
    }

    public Command setIntakeVelocityCommand(double velocity) {
        return runEnd(
            ()-> {
                intakeSetVelocity(velocity);
            },
            () -> {
                intakeMotor.set(0);
            }
        );
    }

    public Command setSweeperSpeedCommand(double speed) {
        return runEnd(
            ()-> {
                sweeperMotor.set(speed);
            },
            () -> {
                sweeperMotor.set(0);
            }
        );
    }

    // Use setSweeperSpeedCommand instead unless you know what you're doing!
    public Command setSweeperSpeedCommandNoRequirements(double speed) {
        return Commands.runEnd(
            ()-> {
                sweeperMotor.set(speed);
            },
            () -> {
                sweeperMotor.set(0);
            }
        );
    }

    public Command setBackIntakeSpeedCommandNoRequirements(double speed) {
        return Commands.runEnd(
            ()-> {
                setBackIntakeSpeed(speed);
            },
            () -> {
                setBackIntakeSpeed(0);
            }
        );
    }

    @Override
    public void periodic() {
        log("Note Detected?", isNoteDetected());
        log("Front Mid Beam", frontMidBreak());
        log("Back Mid Beam", backMidBreak());
        log("Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
        log("Motors/Intake Temperature (C)", intakeMotor.getDeviceTemp().getValueAsDouble());
        log("Motors/Sweeper Temperature (C)", sweeperMotor.getMotorTemperature());

        if (pidTuner != null) pidTuner.periodic();
    }
}
